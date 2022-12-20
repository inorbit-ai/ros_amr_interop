# BSD 3-Clause License
#
# Copyright (c) 2022 InOrbit, Inc.
# Copyright (c) 2022 Clearpath Robotics, Inc.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the InOrbit, Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# Import dependencies
import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from vda5050_connector_py.rewritten_yaml import RewrittenYaml


def generate_launch_description():

    # Get the launch directory
    package_name = "vda5050_connector"
    package_dir = get_package_share_directory(package_name)

    # Launch configuration variables for parameters
    namespace = LaunchConfiguration("namespace")
    parameters_config_file = LaunchConfiguration("parameters_config_file")

    # Declare parameters

    declare_namespace = DeclareLaunchArgument(
        "namespace",
        default_value="vda5050",
        description="Namespace to use",
    )

    declare_parameters_config_file_cmd = DeclareLaunchArgument(
        "parameters_config_file",
        default_value=os.path.join(package_dir, "config", "connector_example.yaml"),
        description="Full path to the parameters config file to use",
    )

    # Create our own temporary YAML files that include substitutions
    configured_params = RewrittenYaml(
        source_file=parameters_config_file,
        root_key=namespace,
        param_rewrites={},
        convert_types=False,
    )

    # Nodes

    mqtt_brindge_node = Node(
        package="vda5050_connector",
        executable="mqtt_bridge.py",
        namespace=namespace,
        name="mqtt_bridge",
        parameters=[configured_params],
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare parameters
    ld.add_action(declare_namespace)
    ld.add_action(declare_parameters_config_file_cmd)

    # Launch nodes
    ld.add_action(mqtt_brindge_node)

    return ld
