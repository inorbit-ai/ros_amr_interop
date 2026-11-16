#!/bin/bash
# Run a command (default: bash) in a ROS container, with the checkout mounted
# at /ws/src/ros_amr_interop and /ws as the colcon workspace. The command runs
# as the checkout's owner, not root; see entrypoint.sh.
#
#   SRC=<dir>    mount another tree instead, e.g. a PR worktree
#   IMAGE=<img>  default: ros:humble-ros-base, as in CI
set -e
here=$(dirname "$(realpath "$0")")
# -e: docker would create a missing dir as root, and run as root in it.
src=$(realpath -e "${SRC:-$here/../..}")
if [ -t 0 ]; then tty=-it; else tty=-i; fi

# -u 0: the entrypoint must start as root, even if the image sets USER.
# --init: forwards Ctrl-C/SIGTERM, so the container stops with the command.
exec docker run --rm --init $tty -u 0 --hostname ros_amr_interop -w /ws \
  -v "$src":/ws/src/ros_amr_interop \
  -v "$here/entrypoint.sh":/entrypoint.sh:ro --entrypoint /entrypoint.sh \
  "${IMAGE:-ros:humble-ros-base}" "${@:-bash}"
