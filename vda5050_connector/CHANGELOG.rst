^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package vda5050_connector
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.0.0 (2026-11-14)
------------------
* First release for ROS 2 Humble.
  (`#31 <https://github.com/inorbit-ai/ros_amr_interop/issues/31>`_,
  `#33 <https://github.com/inorbit-ai/ros_amr_interop/issues/33>`_)
* **Breaking:** order rejections now report ``errorType: "validationError"``
  (VDA 5050 §6.6.4.1) instead of the non-standard ``"validationOrder"``.
  Anything that matches on the old string needs updating.
  (`#83 <https://github.com/inorbit-ai/ros_amr_interop/issues/83>`_)
* **Breaking:** with VDA 5050 2.x, the ``state`` message now carries
  ``information``, the 2.x field name, instead of the 1.x ``informations``.
  The ROS 2 ``OrderState`` message keeps ``informations``; 1.x is unchanged.
  (`#43 <https://github.com/inorbit-ai/ros_amr_interop/issues/43>`_)
* New ``known_map_ids`` parameter: when set, orders whose nodes reference
  any other ``mapId`` are rejected with ``validationError``.
  (`#83 <https://github.com/inorbit-ai/ros_amr_interop/issues/83>`_)
* New ``vda5050_protocol_version`` parameter for the MQTT bridge; topics use
  its major version. (`#53 <https://github.com/inorbit-ai/ros_amr_interop/issues/53>`_)
* New ``enable_nav_through_nodes`` mode: the controller sends the adapter one
  ``NavigateThroughNodes`` goal for several nodes and extends it with
  ``ExtendNavigation`` as the order grows, stopping at nodes with blocking actions.
  (`#75 <https://github.com/inorbit-ai/ros_amr_interop/issues/75>`_,
  `#80 <https://github.com/inorbit-ai/ros_amr_interop/issues/80>`_,
  `#86 <https://github.com/inorbit-ai/ros_amr_interop/issues/86>`_)
* Malformed orders are rejected with ``validationError`` instead of being
  dropped, and a malformed MQTT message no longer stops the bridge.
  (`#84 <https://github.com/inorbit-ai/ros_amr_interop/issues/84>`_,
  `#94 <https://github.com/inorbit-ai/ros_amr_interop/issues/94>`_)
* Orders whose ``nodePosition`` has no ``theta`` are accepted (optional in
  VDA 5050 2.x); the adapter gets NaN.
  (`#93 <https://github.com/inorbit-ai/ros_amr_interop/issues/93>`_)
* Serial numbers that aren't valid ROS 2 names (e.g. ``0001``) work; MQTT topics
  keep them unchanged. (`#92 <https://github.com/inorbit-ai/ros_amr_interop/issues/92>`_)
* ``interface_name`` can be configured, and the operating mode comes from the
  adapter's state. (`#60 <https://github.com/inorbit-ai/ros_amr_interop/issues/60>`_)
* ``errorReferences`` keys are camelCase.
  (`#72 <https://github.com/inorbit-ai/ros_amr_interop/issues/72>`_)
* Order handling fixes: stitching uses the last node of the base, action states
  survive stitching, failed navigation goals and failed actions are reported,
  a canceled order's errors are cleared, and instant actions don't count as an
  active order.
  (`#42 <https://github.com/inorbit-ai/ros_amr_interop/issues/42>`_,
  `#45 <https://github.com/inorbit-ai/ros_amr_interop/issues/45>`_,
  `#60 <https://github.com/inorbit-ai/ros_amr_interop/issues/60>`_,
  `#64 <https://github.com/inorbit-ai/ros_amr_interop/issues/64>`_,
  `#65 <https://github.com/inorbit-ai/ros_amr_interop/issues/65>`_,
  `#68 <https://github.com/inorbit-ai/ros_amr_interop/issues/68>`_,
  `#69 <https://github.com/inorbit-ai/ros_amr_interop/issues/69>`_,
  `#76 <https://github.com/inorbit-ai/ros_amr_interop/issues/76>`_)
* Edge trajectory control points keep their orientation.
  (`#63 <https://github.com/inorbit-ai/ros_amr_interop/issues/63>`_)
* MQTT bridge: retries until the first broker connection succeeds, exits cleanly
  on SIGINT, and logs connection errors without raising.
  (`#55 <https://github.com/inorbit-ai/ros_amr_interop/issues/55>`_,
  `#70 <https://github.com/inorbit-ai/ros_amr_interop/issues/70>`_,
  `#73 <https://github.com/inorbit-ai/ros_amr_interop/issues/73>`_,
  `#85 <https://github.com/inorbit-ai/ros_amr_interop/issues/85>`_)

1.1.1 (2022-10-28)
------------------
* Fixed ROS buildfarm job (`#29 <https://github.com/inorbit-ai/ros_amr_interop/issues/29>`_)

1.1.0 (2022-10-17)
------------------
* Added VDA5050 connector package (`#28 <https://github.com/inorbit-ai/ros_amr_interop/issues/28>`_)
