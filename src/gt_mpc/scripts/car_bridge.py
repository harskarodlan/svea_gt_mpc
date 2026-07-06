#! /usr/bin/env python3



from svea_core.interfaces import LocalizationInterface
from svea_core.interfaces import ActuationInterface, ShowMarker, ShowPath
from svea_core import rosonic as rx

from std_msgs.msg import Float64MultiArray


class CarBridgeNode(rx.Node):
    
    DELTA_TIME = 0.1

    # Interfaces
    
    actuation = ActuationInterface(use_acceleration=True)
    localizer = LocalizationInterface()

    # publish state to eg. svea_a/gt_mpc/state
    state_pub = rx.Publisher(Float64MultiArray, 'gt_mpc/state')

    def on_startup(self):
        self.create_timer(self.DELTA_TIME, self.publish_state)

    def publish_state(self):
        state = self.localizer.get_state()
        x, y, yaw, vel = state

        state_msg = Float64MultiArray()
        state_msg.data = [x, y, yaw, vel]
        self.state_pub.publish(state_msg)
    
    # Subscribe to control eg. svea_a/gt_mpc/control
    # and send to actuation
    @rx.Subscriber(Float64MultiArray, 'gt_mpc/control')
    def send_control(self, msg):
        steering, velocity = msg.data
        # self.get_logger().info(f"Steering: {steering}, Velocity: {velocity}")
        self.actuation.send_control(steering, velocity)


if __name__ == '__main__':
    CarBridgeNode.main()
