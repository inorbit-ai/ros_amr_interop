![InOrbit + Open-RMF](assets/open%20rmf%20inorbit%20github%20header%20narrow%202.png)

Full Control Fleet Adapter for integrating InOrbit with [Robotics Middleware Framework](https://github.com/open-rmf/rmf#robotics-middleware-framework-rmf) (RMF).

## Overview

The package includes the RMF InOrbit Fleet Adapter, which enables communication between a fleet of robots controlled by InOrbit and RMF core, allowing centralized coordinated control of a fleet of multiple robots. It utilizes the InOrbit REST API to communicate with InOrbit, while the adapter and RMF run on ROS2 in different nodes.
Demos and a template can be found in the [`rmf_inorbit_examples`](https://github.com/inorbit-ai/rmf_inorbit_examples) repository, containing a usage demonstration of the InOrbit fleet adapter in a simulated environment and a base configuration to be modified for a particular scenario.
Each instance of this adapter will work for a fleet in one location. For multiple locations, like offices in different buildings, different instances of the adapter have to be configured and run independently. For each adapter modify the [template package](https://github.com/inorbit-ai/rmf_inorbit_examples/tree/main/rmf_inorbit_template) following the instructions in it.

![mission video](assets/full%20mission.gif)

## Features

This package allows sending tasks to a heterogeneous fleet of robots with the same capabilities in the same location controlled via InOrbit. It supports loop tasks for demonstration purposes and as a proof of concept.
It does not currently include the following functionality:

- Dispatching InOrbit actions such as docking
- Tasks other than Loops, such as cleaning or delivery tasks

## Environment

ROS2 Humble + Ubuntu 22.04 / Docker

## Setup

To utilize the fleet adapter an InOrbit account must be set up first. To run a demo simulation, take a look at the documentation of the [`rmf_inorbit_demos`](https://github.com/inorbit-ai/rmf_inorbit_examples/tree/main/rmf_inorbit_demos) package at [`rmf_inorbit_examples`](https://github.com/inorbit-ai/rmf_inorbit_examples) for instructions, and to use the fleet adapter in your own environment, visit [`rmf_inorbit_template`](https://github.com/inorbit-ai/rmf_inorbit_examples/tree/main/rmf_inorbit_template) in the same repository.

## How to use it

The package is prepared to be run in a dockerized environment, but it can also be run on a non docker box.

### Workspace setup

Create the workspace directory tree in your host machine before running the container:

```
mkdir -p inorbit_rmf_ws/src
cd inorbit_rmf_ws/src
```

#### Build and run the docker environment

Run the following script to build the docker image and run a container:

```
cd .ci/docker
./start_local_dev.sh
```

Install dependencies:

```
# Required:
python3 -m pip install requests

# If you want to be able to edit traffic maps:
sudo apt update && sudo apt install -y \
    ros-humble-rmf-traffic-editor \
    ros-humble-rmf-building-map-tools
```

#### Non docker setup

If you don't want to use docker, ignore the previous two steps and:

1. In a Ubuntu 22.04 box, install ROS2 Humble following these [instructions](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html).
2. Install dependencies:

```
# Required:
python3 -m pip install requests

# If you want to be able to edit traffic maps:
sudo apt update && sudo apt install -y \
    ros-humble-rmf-traffic-editor \
    ros-humble-rmf-building-map-tools
```

3. Source ROS2 Humble installation

```
source /opt/ros/humble/setup.bash
```

### Project build

To build the packages, inside the container (at `~/ws`) run:

```
rosdep install --from-paths src --rosdistro humble -y --ignore-src
colcon build
```

A warning about the deprecation of `setup.py` will pop up. This is a known ROS issue, and you can choose to ignore this warning by adding the following environment variable:

```
echo 'PYTHONWARNINGS="ignore:setup.py install is deprecated::setuptools.command.install"; export PYTHONWARNINGS' >> ~/.bashrc
```

### Test

After building your package, you can run tests by:

```
colcon test
colcon test-result
```

## Run the adapter

Remember to source the overlay on every new bash session:

```
. install/setup.bash
```

The demos and template packages include examples of launch files for the adapter and RMF, as well as instructions for how to set up the adapter configuration.
The fleet adapter package contains one launch file for the adapter alone, that takes the following arguments:

```
$ ros2 launch rmf_inorbit_fleet_adapter rmf_inorbit_fleet_adapter.launch.xml --show-args
Arguments (pass arguments as '<name>:=<value>'):

    'api_key':
        InOrbit API key

    'adapter_config_file':
        Path to the configuration file of the adapter

    'nav_graph_file':
        Path to the navigation graph file for RMF

```

To launch just the adapter (note that it will eventually stop if the underlying infrastructure is not running):

```
ros2 launch rmf_inorbit_fleet_adapter rmf_inorbit_fleet_adapter.launch.xml api_key:=<InOrbit API KEY> adapter_config_file:=<path to configuration file> nav_graph_file:=<path to navigation graph file>
```

It will start the following:

#### Nodes

- `/{fleet name}_fleet_adapter`: Instance of the full control fleet adapter node rmf_adapter provides.
- `/inorbit_fleet_command_handle`: Implementation of rmf_adapter.RobotCommandHandle

All of the topics and services are part of the mentioned nodes. For more information about them, visit [rmf_ros2](https://github.com/open-rmf/rmf_ros2).

#### Topics

```
/dispenser_requests
/dispenser_results
/dispenser_states
/dock_summary
/fleet_states
/ingestor_requests
/ingestor_results
/ingestor_states
/lane_states
/nav_graphs
/rmf_traffic/query_update_1
/task_summaries
```

#### Services

```
/<InOrbitSite>_fleet_adapter/describe_parameters
/<InOrbitSite>_fleet_adapter/get_parameter_types
/<InOrbitSite>_fleet_adapter/get_parameters
/<InOrbitSite>_fleet_adapter/list_parameters
/<InOrbitSite>_fleet_adapter/set_parameters
/<InOrbitSite>_fleet_adapter/set_parameters_atomically
/inorbit_fleet_command_handle/describe_parameters
/inorbit_fleet_command_handle/get_parameter_types
/inorbit_fleet_command_handle/get_parameters
/inorbit_fleet_command_handle/list_parameters
/inorbit_fleet_command_handle/set_parameters
/inorbit_fleet_command_handle/set_parameters_atomically
```

### Parameters

```
/<InOrbitSite>_fleet_adapter:
    discovery_timeout
    qos_overrides./parameter_events.publisher.depth
    qos_overrides./parameter_events.publisher.durability
    qos_overrides./parameter_events.publisher.history
    qos_overrides./parameter_events.publisher.reliability
    use_sim_time
/inorbit_fleet_command_handle:
    use_sim_time
```

## Configuration

Visit [`rmf_inorbit_examples`](https://github.com/inorbit-ai/rmf_inorbit_examples) to get access to the demos and the template configuration package.

## Contributing

Please see the [CONTRIBUTING](CONTRIBUTING.md) document.

## License

[![License](https://img.shields.io/badge/License-BSD_3--Clause-blue.svg)](LICENSE)

![Powered by InOrbit](assets/open%20rmf%20inorbit%20github%20footer.png)
