# Test samples

Misc commands for various tests

## Publishing ROS2 messages manually

### Path

```bash
ros2 topic pub --once /we_b_robots/destinations nav_msgs/msg/Path \
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
                    "x": 2.0,
                    "y": 0.0,
                    "z": 0.0
                },
                "orientation": {
                    "x": 0.0,
                    "y": 0.0,
                    "z": 1.8,
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
                    "x": 2.0,
                    "y": 1.0,
                    "z": 0.0
                },
                "orientation": {
                    "x": 0.0,
                    "y": 0.0,
                    "z": 1.8,
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
                    "x": 3.0,
                    "y": 1.0,
                    "z": 0.0
                },
                "orientation": {
                    "x": 0.0,
                    "y": 0.0,
                    "z": 1.8,
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
                    "x": 2.0,
                    "y": 4.0,
                    "z": 0.0
                },
                "orientation": {
                    "x": 1.0,
                    "y": 1.0,
                    "z": 1.8,
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
                    "x": 2.0,
                    "y": 6.0,
                    "z": 0.0
                },
                "orientation": {
                    "x": 0.0,
                    "y": 2.0,
                    "z": 1.8,
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
                    "x": 6.2,
                    "y": 2.0,
                    "z": 0.0
                },
                "orientation": {
                    "x": 0.0,
                    "y": 2.0,
                    "z": 1.8,
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
                    "x": 11.0,
                    "y": 0.0,
                    "z": 0.0
                },
                "orientation": {
                    "x": 0.0,
                    "y": 4.0,
                    "z": 1.8,
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
                    "x": 9.0,
                    "y": 5.0,
                    "z": 0.0
                },
                "orientation": {
                    "x": 0.0,
                    "y": 1.0,
                    "z": 1.8,
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
                    "x": 6.0,
                    "y": 3.1,
                    "z": 0.0
                },
                "orientation": {
                    "x": 0.0,
                    "y": 2.0,
                    "z": 1.8,
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
                    "x": 1.0,
                    "y": 1.0,
                    "z": 0.0
                },
                "orientation": {
                    "x": 0.0,
                    "y": 0.0,
                    "z": 1.8,
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
                    "x": 5.3,
                    "y": 3.0,
                    "z": 0.0
                },
                "orientation": {
                    "x": 0.0,
                    "y": 0.0,
                    "z": 1.8,
                    "w": 1
                }
            }
        }
    ]
}'
```

### TwistStamped

```bash
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
```

### BatteryState

```bash
ros2 topic pub --once /good_sensors/bat sensor_msgs/msg/BatteryState '
{
    percentage: 91.3
}'
```

### PoseStamped

```bash
ros2 topic pub --once /move_base_simple/goal geometry_msgs/msg/PoseStamped '
{
    "header": {
        "frame_id": "floor1"
    },
    "pose": {
        "position": {
            "x": 2,
            "y": 0,
            "z": 0
        },
        "orientation": {
            "x": 0,
            "y": 0,
            "z": 1.8,
            "w": 1
        }
    }
}'
```
