# Sample data for massrobotics_amr_sender

Scripts, launch files, recordings and other tools for demoing and testing the `massrobotics_amr_sender` node.

The `rosbag` folder contains a stripped rosbag based on `turtlebot3`. It was built using on the [Gazebo](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#gazebo-simulation), [SLAM](https://emanual.robotis.com/docs/en/platform/turtlebot3/slam_simulation/) and [Navigation](https://emanual.robotis.com/docs/en/platform/turtlebot3/nav_simulation/) simulations.

```bash
$ rosbag info rosbag_demo.bag
path:        rosbag_demo.bag
version:     2.0
duration:    1:51s (111s)
start:       Jul 05 2021 19:45:30.36 (1625525130.36)
end:         Jul 05 2021 19:47:21.75 (1625525241.75)
size:        3.8 MB
messages:    12282
compression: none [4/4 chunks]
types:       geometry_msgs/PoseStamped  [d3812c3cbc69362b77dc0b19b345f8f5]
             geometry_msgs/TwistStamped [98d34b0043a2093cf9d9345ab6eef12e]
             nav_msgs/Path              [6227e2b7e9cce15051f669a5e197bbf7]
             sensor_msgs/BatteryState   [4ddae7f048e32fda22cac764685e3974]
             std_msgs/Float32           [73fcbf46b49191e672908e50842a83d4]
             std_msgs/String            [992ce8a1687cec8c8bd883ec73ca41d1]
topics:      /battery                       111 msgs    : sensor_msgs/BatteryState
             /battery_runtime                37 msgs    : std_msgs/Float32
             /load_perc_available            22 msgs    : std_msgs/Float32
             /local_plan                   1453 msgs    : nav_msgs/Path
             /location                     5273 msgs    : geometry_msgs/PoseStamped
             /mode                            5 msgs    : std_msgs/String
             /plan                           74 msgs    : nav_msgs/Path
             /troubleshooting/errorcodes     37 msgs    : std_msgs/String
             /velocity                     5270 msgs    : geometry_msgs/TwistStamped
```

Messages on topics such as `/plan` and `/local_plan` were kept unchanged while messages on `/location` and `/velocity` were crafted by creating `PoseStamped` and `TwistStamped` messages using data from `Odometry` messages on topic `/odom`. The messages on the remaining topics `/battery`, `/battery_runtime`, `/load_perc_available`, `/mode` and `/troubleshooting/errorcodes` as well as all the transformation described above were generated with a small ROS1 node that is available at `synthetic/node.py`.

## How to run

The `massrobotics_amr_sender_rosbag_launch.launch` launch file describes a `massrobotics_amr_sender` node that uses a configuration file customized for the sample rosbag, and also plays the rosbag in loop mode so the different node callbacks are executed.

```bash
roslaunch massrobotics_amr_sender massrobotics_amr_sender_rosbag_launch.launch
```
