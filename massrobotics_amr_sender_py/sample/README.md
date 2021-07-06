# Sample data for massrobotics_amr_sender

Scripts, launch files, recordings and other tools for demoing and testing the `massrobotics_amr_sender` node.

The `rosbag` folder contains a stripped rosbag based on `turtlebot3`. I was built relying on the [Gazebo](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#gazebo-simulation), [SLAM](https://emanual.robotis.com/docs/en/platform/turtlebot3/slam_simulation/) and [Navigation](https://emanual.robotis.com/docs/en/platform/turtlebot3/nav_simulation/) simulations.

```bash
$ ros2 bag info rosbag/rosbag_demo.db3
[INFO] [1625526486.600008195] [rosbag2_storage]: Opened database 'rosbag_demo.db3' for READ_ONLY.

Files:             rosbag/rosbag_demo.db3
Bag size:          4.2 MiB
Storage id:        sqlite3
Duration:          111.390s
Start:             Jul  5 2021 19:45:30.357 (1625525130.357)
End:               Jul  5 2021 19:47:21.747 (1625525241.747)
Messages:          12282
Topic information: Topic: /battery | Type: sensor_msgs/msg/BatteryState | Count: 111 | Serialization Format: cdr
                   Topic: /battery_runtime | Type: std_msgs/msg/Float32 | Count: 37 | Serialization Format: cdr
                   Topic: /load_perc_available | Type: std_msgs/msg/Float32 | Count: 22 | Serialization Format: cdr
                   Topic: /local_plan | Type: nav_msgs/msg/Path | Count: 1453 | Serialization Format: cdr
                   Topic: /location | Type: geometry_msgs/msg/PoseStamped | Count: 5273 | Serialization Format: cdr
                   Topic: /mode | Type: std_msgs/msg/String | Count: 5 | Serialization Format: cdr
                   Topic: /plan | Type: nav_msgs/msg/Path | Count: 74 | Serialization Format: cdr
                   Topic: /troubleshooting/errorcodes | Type: std_msgs/msg/String | Count: 37 | Serialization Format: cdr
                   Topic: /velocity | Type: geometry_msgs/msg/TwistStamped | Count: 5270 | Serialization Format: cdr
```

Messages on topics such as `/plan` and `/local_plan` were kept unchanged while messages on `/location` and `/velocity` were crafted by creating `PoseStamped` and `TwistStamped` messages using data from `Odometry` messages on topic `/odom`. The messages on the remaning topics `/battery`, `/battery_runtime`, `/load_perc_available`, `/mode` and `/troubleshooting/errorcodes` as well as all the transformation described above were generated with very basic ROS2 node that is available at `synthetic/node.py`.

## How to run

The `massrobotics_amr_sender_rosbag_launch.py` launch file describes a `massrobotics_amr_sender` node that is configured to use a configuration file that is specific for the sample rosbag, and also plays the rosbag in loop mode so the different node callbacks are executed.

```bash
ros2 launch massrobotics_amr_sender_rosbag_launch.py
```
