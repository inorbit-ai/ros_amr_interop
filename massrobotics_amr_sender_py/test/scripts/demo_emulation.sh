#!/bin/bash

publish_initial_pose() {
    ros2 topic pub --once /move_base_simple/goal geometry_msgs/msg/PoseStamped '
{
    "header": {
        "frame_id": "floor1"
    },
    "pose": {
        "position": {
            "x": 10,
            "y": 10,
            "z": 0
        },
        "orientation": {
            "x": 0,
            "y": 0,
            "z": 0,
            "w": 1
        }
    }
}'

}
publish_path() {
    ros2 topic pub --once /magic_nav/path nav_msgs/msg/Path \
'{
    "header": {
        "frame_id": "floor1"
    },
    "poses": [
        {
            "header": {
                "frame_id": "floor1"
            },
            "pose": {
                "position": {
                    "x": 10,
                    "y": 10,
                    "z": 0.0
                },
                "orientation": {
                    "x": 0.0,
                    "y": 0.0,
                    "z": 0.0,
                    "w": 1
                }
            }
        },
        {
            "header": {
                "frame_id": "floor1"
            },
            "pose": {
                "position": {
                    "x": 19.0,
                    "y": 11.0,
                    "z": 0.0
                },
                "orientation": {
                    "x": 0.0,
                    "y": 0.0,
                    "z": 0.0,
                    "w": 1
                }
            }
        },
        {
            "header": {
                "frame_id": "floor1"
            },
            "pose": {
                "position": {
                    "x": 20.0,
                    "y": 15.0,
                    "z": 0.0
                },
                "orientation": {
                    "x": 0.0,
                    "y": 0.0,
                    "z": 0.0,
                    "w": 1
                }
            }
        },
        {
            "header": {
                "frame_id": "floor1"
            },
            "pose": {
                "position": {
                    "x": 30.0,
                    "y": 15.0,
                    "z": 0.0
                },
                "orientation": {
                    "x": 0.0,
                    "y": 0.0,
                    "z": 0.0,
                    "w": 1
                }
            }
        },

    ]
}'
}

publish_pose_1() {
    ros2 topic pub --once /move_base_simple/goal geometry_msgs/msg/PoseStamped '
{
    "header": {
        "frame_id": "floor1"
    },
    "pose": {
        "position": {
            "x": 19.0,
            "y": 11.0,
            "z": 0
        },
        "orientation": {
            "x": 0,
            "y": 0,
            "z": 0,
            "w": 1
        }
    }
}'

ros2 topic pub --once /good_sensors/vel geometry_msgs/msg/TwistStamped '
{
    "header": {
        "frame_id": "floor1"
    },
    "twist": {
        "linear": {
            "x": 1,
            "y": 2,
            "z": 3
        },
        "angular": {
            "x": 1,
            "y": 1,
            "z": 1
        }
    }
}'

}

publish_pose_2() {
    ros2 topic pub --once /move_base_simple/goal geometry_msgs/msg/PoseStamped '
{
    "header": {
        "frame_id": "floor1"
    },
    "pose": {
        "position": {
            "x": 20.0,
            "y": 15.0,
            "z": 0
        },
        "orientation": {
            "x": 0,
            "y": 0,
            "z": 0,
            "w": 1
        }
    }
}'

ros2 topic pub --once /good_sensors/vel geometry_msgs/msg/TwistStamped '
{
    "header": {
        "frame_id": "floor1"
    },
    "twist": {
        "linear": {
            "x": 5,
            "y": 2,
            "z": 3
        },
        "angular": {
            "x": 3,
            "y": 3,
            "z": 3
        }
    }
}'

}

publish_pose_3() {
    ros2 topic pub --once /move_base_simple/goal geometry_msgs/msg/PoseStamped '
{
    "header": {
        "frame_id": "floor1"
    },
    "pose": {
        "position": {
            "x": 30.0,
            "y": 15.0,
            "z": 0
        },
        "orientation": {
            "x": 0,
            "y": 0,
            "z": 0,
            "w": 1
        }
    }
}'

ros2 topic pub --once /good_sensors/vel geometry_msgs/msg/TwistStamped '
{
    "header": {
        "frame_id": "floor1"
    },
    "twist": {
        "linear": {
            "x": 1,
            "y": 0,
            "z": 0
        },
        "angular": {
            "x": 0,
            "y": 1,
            "z": 1
        }
    }
}'

}

clear_path() {
    ros2 topic pub --once /magic_nav/path nav_msgs/msg/Path
}

clear_path
sleep 5

for i in {1..3}
do
    publish_initial_pose
    publish_path
    sleep 3
    publish_pose_1
    sleep 3
    publish_pose_2
    sleep 3
    publish_pose_3
    sleep 3
    clear_path
    sleep 5
done
