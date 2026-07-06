## Terminal Instructions for ROS ping test

Open 5 terminals in the docker container in svea ws.

### In terminal 1:

#### Input:
ros2 run gt_mpc overtake.py --ros-args   -p initial_x_a:=0.0 -p initial_y_a:=0.0   -p initial_x_b:=0.0 -p initial_y_b:=0.0

#### Expected output:
[INFO] [1782819056.313128883] [OvertakeNode]: Starting up...
/opt/ros/jazzy/lib/python3.12/site-packages/rclpy/node.py:481: UserWarning: when declaring parameter named 'initial_x_a', declaring a parameter \
... \
warnings.warn( \
OK \
[INFO] [1782819113.010417769] [OvertakeNode]: Overtake node successfully launched! \
[INFO] [1782816673.347686411] [OvertakeNode]: Running... \
OK \
OK 

### In terminal 2:

#### Input:

ros2 topic pub /svea_a/gt_mpc/state std_msgs/msg/Float64MultiArray   "{data: [0.0, 0.0, 0.0, 6.0]}" --rate 10

#### Expected output:

publishing #197: std_msgs.msg.Float64MultiArray(layout=std_msgs.msg.MultiArrayLayout(dim=[], data_offset=0), data=[0.0, 0.0, 0.0, 6.0])


### In terminal 3:

#### Input:

ros2 topic pub /svea_b/gt_mpc/state std_msgs/msg/Float64MultiArray   "{data: [-4.0, 0.0, 0.0, 6.0]}" --rate 10

#### Expected output:

publishing #1166: std_msgs.msg.Float64MultiArray(layout=std_msgs.msg.MultiArrayLayout(dim=[], data_offset=0), data=[-4.0, 0.0, 0.0, 6.0])

### In terminal 4:

#### Input:

ros2 topic echo /svea_a/gt_mpc/control

#### Expected output:

\--- \
layout: \
  dim: [] \
  data_offset: 0 \
data: \
\- 0.0 \
\- 5.981096968825186 \
\---

### In terminal 5:

#### Input:

ros2 topic echo /svea_b/gt_mpc/control

#### Expected output:

\--- \
layout: \
  dim: [] \
  data_offset: 0 \
data: \
\- -0.09919075595326983 \
\- 6.019224557531593 \
\---

