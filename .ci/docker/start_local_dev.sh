#!/bin/bash
# Build the dev image and open a shell in it. The checkout is mounted at
# /ws/src/ros_amr_interop and /ws is the colcon workspace; see run.sh.
set -e
cd "$(dirname "$0")"
DOCKER_BUILDKIT=1 docker build --progress=plain -t ros_amr_interop .
IMAGE=ros_amr_interop exec ./run.sh
