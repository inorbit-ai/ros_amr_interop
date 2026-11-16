#!/bin/bash
# Run a command (default: a shell) in a ROS container, with a source tree
# mounted at /ws/src/ros_amr_interop and /ws as the colcon workspace.
#
# The command runs as the owner of the source tree (any UID), with passwordless
# sudo for apt/rosdep, so it doesn't leave root-owned files on the host. Files
# written with sudo are still root-owned.
#
#   .ci/docker/run.sh [command...]              # this checkout
#   SRC=<dir> .ci/docker/run.sh [command...]    # another tree, e.g. a PR worktree
#   IMAGE=<image> .ci/docker/run.sh ...         # default: ros:humble-ros-base, as in CI
#
# To get a second shell in a running container, pass the same user:
#   docker exec -it -u "$(id -u):$(id -g)" <container> bash
set -e
here=$(cd "$(dirname "$0")" && pwd)
src=$(cd "${SRC:-$here/../..}" && pwd)
if [ -t 0 ]; then tty=-it; else tty=-i; fi

# The entrypoint comes from this checkout (read-only), never from $SRC.
exec docker run --rm --init $tty --user 0:0 --hostname ros_amr_interop -w /ws \
  -v "$src":/ws/src/ros_amr_interop \
  -v "$here/entrypoint.sh":/run_as_owner.sh:ro \
  --entrypoint bash "${IMAGE:-ros:humble-ros-base}" /run_as_owner.sh "${@:-bash}"
