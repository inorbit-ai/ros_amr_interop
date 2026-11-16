#!/bin/bash
# Runs the command as the owner of the mounted checkout instead of root, so
# files it writes there stay deletable on the host. That user gets
# passwordless sudo for apt/rosdep.
set -e
uid=$(stat -c %u /ws/src/ros_amr_interop)
gid=$(stat -c %g /ws/src/ros_amr_interop)

# Create a user with that UID unless the image already has one.
getent group "$gid" >/dev/null || groupadd -g "$gid" owner
getent passwd "$uid" >/dev/null || useradd -m -u "$uid" -g "$gid" owner
echo "#$uid ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/owner
chown "$uid:$gid" /ws

exec setpriv --reuid="$uid" --regid="$gid" --init-groups \
  env HOME="$(getent passwd "$uid" | cut -d: -f6)" /ros_entrypoint.sh "$@"
