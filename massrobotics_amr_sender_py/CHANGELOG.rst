^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package massrobotics_amr_sender
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.0 (2021-06-25)
------------------
* Adding bits for first release (`#7 <https://github.com/inorbit-ai/ros_amr_interop/issues/7>`_)

0.0.2 (2021-06-24)
-------------------
* Changed package name to ``massrobotics_amr_sender``
* Changed repository folder organization

0.0.1 (2021-06-23)
-------------------
* Added support for Identity and Status reports
* Added support for several ROS2 messages
  
  * ``geometry_msgs/TwistStamped``
  * ``sensor_msgs/BatteryState``
  * ``geometry_msgs/PoseStamped``
  * ``nav_msgs/Path``
  * ``std_msgs/String``
  * ``std_msgs/Float32``
  * ``std_msgs/Float64``

* Added unit tests for all message callbacks
* Added ``3-Clause BSD License``
