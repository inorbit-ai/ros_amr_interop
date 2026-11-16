#!/bin/bash
# Entrypoint used by run.sh. Starts as root, then runs the command as the owner
# of the mounted source tree, with passwordless sudo for apt/rosdep. Files the
# command writes into the tree keep the host user's ownership, whatever the UID.
set -e
src=/ws/src/ros_amr_interop
uid=$(stat -c %u "$src")
gid=$(stat -c %g "$src")

# A root-owned tree means the host user is root, or rootless docker/podman maps
# the host user to root. Either way root already is the host user.
[ "$uid" = 0 ] && exec /ros_entrypoint.sh "$@"

getent group "$gid" >/dev/null || groupadd -g "$gid" owner
user=$(getent passwd "$uid" | cut -d: -f1)
if [ -z "$user" ]; then
  user=owner
  useradd -m -u "$uid" -g "$gid" -s /bin/bash "$user"
fi
home=$(getent passwd "$uid" | cut -d: -f6)
[ "$(stat -c %u "$home" 2>/dev/null)" = "$uid" ] || home=/tmp

echo "$user ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/run-as-owner
chown "$uid:$gid" /ws /ws/src
exec setpriv --reuid="$uid" --regid="$gid" --init-groups \
  env HOME="$home" USER="$user" /ros_entrypoint.sh "$@"
