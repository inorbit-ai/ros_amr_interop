# massrobotics_amr_sender

Configuration-based ROS2 package for sending MassRobotics [AMR Interop Standard messages](https://github.com/MassRobotics-AMR/AMR_Interop_Standard) to complaint receivers.

## Running from source

Make sure `ros2` is installed properly. Clone this repository inside your `src` folder on your local workspace and build the package:

```bash
mkdir -p ~/ros2_ws/src && cd ros2_ws/
git clone https://github.com/inorbit-ai/ros_amr_interop.git ./src

colcon build --packages-select massrobotics_amr_sender
```

## Running the node from source

The node takes the MassRobotics AMR config file path as parameter. If not provided, it is assumed the file is on the current directory.

```bash
# Source the local overlay by running `. install/setup.sh` if
# using bash or `. install/setup.zsh` if using zsh.
# Also, install dependencies by running `rosdep install --ignore-src --from-paths src/`
ros2 run massrobotics_amr_sender massrobotics_amr_node \
    --ros-args -p config_file:=/path/to/config.yaml --log-level debug
```

## Running tests

On you local workspace:

```bash
colcon test --packages-select massrobotics_amr_sender
colcon test-result --verbose
```

## Releasing a new version

This is a summary of the steps required for releasing a new package.

1. Run ``catkin_generate_changelog`` to generate or update CHANGELOG.rst file(s)

    ```bash
    # Change directory to repository root folder
    $ cd ../
    $ catkin_generate_changelog
    Found packages: massrobotics_amr_sender
    Querying commit information since latest tag...
    Updating forthcoming section of changelog files...
    - updating 'massrobotics_amr_sender_py/CHANGELOG.rst'
    Done.
    Please review the extracted commit messages and consolidate the changelog entries before committing the files!
    ```

2. Open ``CHANGELOG.rst`` and edit to your liking. Then commit your new/updated changelog.

3. Run ``catkin_prepare_release`` for bumping up package version. By default this command increases the patch version of your package, e.g. ``0.1.1 -> 0.1.2``, but you can pick minor or major using the ``--bump`` option.

    **Note**: this command increment the version in your ``package.xml``'s, and commit/tag the changes with a bloom compatible flag.

https://docs.ros.org/en/foxy/Contributing/Developer-Guide.html#library-versioning