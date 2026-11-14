^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package vda5050_msgs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.0.0 (2026-11-14)
------------------
* **Breaking:** ``NodeState.position`` is now ``node_position`` and
  ``OrderState.new_base_requested`` is now ``new_base_request``, the VDA 5050
  field names. ``CurrentAction`` gains ``action_type``, and ``Trajectory.degree``
  defaults to 1.0. (`#42 <https://github.com/inorbit-ai/ros_amr_interop/issues/42>`_)

1.1.1 (2022-10-28)
------------------
* Fixed ROS buildfarm job (`#29 <https://github.com/inorbit-ai/ros_amr_interop/issues/29>`_)
