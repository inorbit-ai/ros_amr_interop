# ROS AMR interoperability packages

[![black](https://img.shields.io/badge/code%20style-black-000000.svg)](https://github.com/psf/black)

This repository hosts a collection of ROS packages to ease
the integration of ROS based robots with different interoperability
standards, with a focus on AMRs (Autonomous Mobile Robots).

## Packages

The following packages are included in this repository:


### Mass Robotics AMR Interop Sender for ROS2

The [massrobotics_amr_sender_py](https://github.com/inorbit-ai/ros_amr_interop/tree/foxy-devel/massrobotics_amr_sender_py#readme)
package provides a ROS2 node written in Python that takes input from a
ROS2 system and publishes it to a [Mass Robotics Interop compliant
Receiver](https://github.com/MassRobotics-AMR/AMR_Interop_Standard/tree/main/MassRobotics-AMR-Receiver).

Mapping of different data elements from the ROS2 system into Mass
Robotics Interop messages can be customized through a YAML configuration
file.

## Related Initiatives

The topic of AMR interoperability is in a fluid state of evolution. For this reason, it is worth it to keep track of other standards, initiatives, libraries and efforts related to this topic.

The following is an incomplete and growing list of such related topics:

 * [Open-RMF](https://osrf.github.io/ros2multirobotbook/) (formerly RMF Core) is an
 open source framework based on ROS 2 to enable the interoperability of heterogeneous
 fleets of any type of robotics systems.
 * [Mass Robotics AMR Interoperability Standard](https://github.com/MassRobotics-AMR/AMR_Interop_Standard) aims to help organizations deploy AMRs from multiple vendors and have them coexist effectively.
 * [VDA 5050](https://www.vda.de/en/services/Publications/vda-5050-v-1.1.-agv-communication-interface.html)
 AGV Communications Interface describes an interface for communication between driverless
 transport vehicles (AGV) and a master control system over MQTT using standardized
 JSON messages.
 * [OPC Unified Architecture](https://opcfoundation.org/about/opc-technologies/opc-ua/)
   (OPC UA) is a machine to machine communication protocol for industrial
 automation developed by the OPC Foundation
   * [ros_opcua_communication](http://wiki.ros.org/ros_opcua_communication) ROS bindings for different open-source OPC-UA implementations

We expect to keep curating the set of relevant topics with the contribution of the community.


## Development

Install [pre-commit](https://pre-commit.com/) in your computer and then set it up by running `pre-commit install` at the root of the cloned project.
