#!/usr/bin/env python3

import numpy as np

from math import cos, sin

from svea_core import rosonic as rx

from gt_mpc.controllers.game_controller import GameController

from std_msgs.msg import Float64MultiArray

from foxglove_msgs.msg import PoseInFrame

import rclpy
from rclpy.qos import QoSProfile
qos_subber = QoSProfile(depth=10 # Size of the queue 
                        )

class OvertakeNode(rx.Node):

    DELTA_TIME = 0.1
    MIN_DIST = 0.3 # threshold for vehicle gap before overtaking
    ROAD_HEADING = 0.9 # [rad] angle giving direction of road
 
    # coordinates for reference point on road
    ROAD_X = None 
    ROAD_Y = None

    # Control publishers for each car
    ctrl_pub_a = rx.Publisher(Float64MultiArray, 'svea_a/gt_mpc/control')
    ctrl_pub_b = rx.Publisher(Float64MultiArray, 'svea_b/gt_mpc/control')

    # get initial positions from param in launch file
    initial_pose_x_a = rx.Parameter(0.0)
    initial_pose_y_a = rx.Parameter(0.0)
    initial_pose_x_b = rx.Parameter(0.0)
    initial_pose_y_b = rx.Parameter(0.0)

    def __init__(self, controller: GameController):
        self.controller = controller
        super().__init__()

    # To get coordinates of cursor click in Foxglove
    @rx.Subscriber(PoseInFrame, '/move_base_simple/goal', qos_subber)
    def ctrl_request_twist(self, twist_msg):
        pass

    def on_startup(self):
        self.state_a = None # [x, y, yaw, vel] published from car_bridge
        self.state_b = None

        self.get_logger().info(f"initial_pose_x_a: {self.initial_pose_x_a}")

        # set reference point on road as average of both
        # vehicles starting positions
        self.ROAD_X = (self.initial_pose_x_a + self.initial_pose_x_b)/2
        self.ROAD_Y = (self.initial_pose_y_a + self.initial_pose_y_b)/2

        self.create_timer(self.DELTA_TIME, self.loop)

        self.get_logger().info("Overtake node successfully launched!")

    # Subscribe to SVEA-A's state
    @rx.Subscriber(Float64MultiArray, 'svea_a/gt_mpc/state')
    def get_state_a(self, msg):
        self.state_a = msg.data

    # Subscribe to SVEA-B's state
    @rx.Subscriber(Float64MultiArray, 'svea_b/gt_mpc/state')
    def get_state_b(self, msg):
        self.state_b = msg.data

    def pos_wrt_road(self, x, y):
        # TO BE IMPLEMENTED
        p = y
        l = x
        return p, l


    def loop(self):
        if self.state_a is None or self.state_b is None:
            self.get_logger().info("No state yet!")
            return

        # get state [x, y, yaw, vel] for each vehicle
        x1, y1, _, v1 = self.state_a
        x2, y2, _, v2 = self.state_b

        self.get_logger().info("State A: x1: {:.2f}, y1: {:.2f}, v1: {:.2f}".format(x1, y1, v1))
        self.get_logger().info("State B: x2: {:.2f}, y2: {:.2f}, v2: {:.2f}".format(x2, y2, v2))

        # get longitudinal (p) (along road) and 
        # lateral (l) (across road) positions wrt road
        p1, l1 = self.pos_wrt_road(x1, y1)
        p2, l2 = self.pos_wrt_road(x2, y2)

        # TEMPORARY HARDCODED STATES FOR PING TESTING
        #p1, l1 = 0.0, 0.5
        #p2, l2 = -4.0, 0.5 

        # decide wether overtaking based on longitudinal distance between cars
        case = 'Overtake' if (p1 - p2) < self.MIN_DIST else 'Platoon'
        self.get_logger().info(case)

        u1, u2 = self.controller.compute_control([p1, v1, l1, p2, v2, l2], case)

        self.get_logger().info("Control A: accel1: {:.2f}, steer1: {:.2f}".format(u1[0], u1[1]))
        self.get_logger().info("Control B: accel2: {:.2f}, steer2: {:.2f}".format(u2[0], u2[1]))

        # convert acceleration control to velocity
        #u1_vel = v1 + u1[0]*self.DELTA_TIME
        #u2_vel = v2 + u2[0]*self.DELTA_TIME

        # Publish controls for SVEA-A
        ctrl_msg_a = Float64MultiArray()
        ctrl_msg_a.data = [u1[1], u1[0]]
        self.ctrl_pub_a.publish(ctrl_msg_a)

        # Publish controls for SVEA-B
        ctrl_msg_b = Float64MultiArray()
        ctrl_msg_b.data = [u2[1], u2[0]]
        self.ctrl_pub_b.publish(ctrl_msg_b)


def main(args=None):
    print("Starting GameController warmup before ROS node initialization...")
    controller = GameController()
    print("GameController warmup complete. Initializing ROS and launching node...")

    rclpy.init(args=args)
    node = OvertakeNode(controller)

    logger = node.get_logger()
    logger.info("Overtake launched...")
    logger.info("Starting up...")
    node.__rosonic_startup__(node)
    logger.info("Running...")

    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        logger.info("Shutting down...")
        node.__rosonic_shutdown__(node)
        rclpy.shutdown()


if __name__ == '__main__':
    main()
