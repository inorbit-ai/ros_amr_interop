#!/bin/bash

DOCKER_BUILDKIT=1 docker build --progress=plain -t ros_amr_interop .

REPO_ROOT="$(git rev-parse --show-toplevel)"

docker run --rm -ti \
    --hostname ros_amr_interop \
    -v ${REPO_ROOT}:/home/docker/dev_ws/src \
    ros_amr_interop
