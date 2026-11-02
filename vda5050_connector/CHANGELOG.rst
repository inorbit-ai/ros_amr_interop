^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package vda5050_connector
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* **Breaking:** order rejections now report ``errorType: "validationError"``
  (VDA 5050 §6.6.4.1) instead of the non-standard ``"validationOrder"``.
  Anything that matches on the old string needs updating.
  (`#83 <https://github.com/inorbit-ai/ros_amr_interop/issues/83>`_)
* New ``known_map_ids`` parameter: when set, orders whose nodes reference
  any other ``mapId`` are rejected with ``validationError``.
  (`#83 <https://github.com/inorbit-ai/ros_amr_interop/issues/83>`_)

1.1.1 (2022-10-28)
------------------
* Fixed ROS buildfarm job (`#29 <https://github.com/inorbit-ai/ros_amr_interop/issues/29>`_)

1.1.0 (2022-10-17)
------------------
* Added VDA5050 connector package (`#28 <https://github.com/inorbit-ai/ros_amr_interop/issues/28>`_)
