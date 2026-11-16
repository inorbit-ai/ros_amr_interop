# Notes for coding agents

## Docker: never leave root-owned files on the host

Containers run as root by default. Anything a root container writes to a host
bind mount ends up owned by root on the host: `__pycache__`, `.pytest_cache`,
colcon `build/` `install/` `log/`, logs, output dirs. A non-root user then
can't delete the worktree or scratch dir.

Use `.ci/docker/run.sh` for anything that mounts a source tree. It runs the
command as the owner of the tree (any UID), with passwordless sudo for
apt/rosdep, and `SRC=<dir>` points it at another tree such as a PR worktree.
See the header of `.ci/docker/run.sh` for usage.

If you run a container any other way:

- Never give a root container a writable bind mount. Mount `:ro` and work in
  dirs that exist only inside the container, or pass
  `--user "$(id -u):$(id -g)" -e HOME=/tmp`. The `ros:*` images have no
  matching user, so with `--user` apt/rosdep won't work.
- `docker exec` runs as the container's start user: root by default, and
  root for `run.sh` containers. Pass `-u "$(id -u):$(id -g)"`.
- `sudo` inside the container writes as root. Don't use it on mounted paths.
- Prefer one-shot `docker run --rm`. If you start a long-lived container,
  `docker rm -f` it when you're done.

Already stuck? Hand the files back without sudo:

```bash
docker run --rm -v "<dir>":/d alpine find /d -xdev -user 0 -exec chown -h "$(id -u):$(id -g)" {} +
```

## Testing

CI (`.github/workflows/tests.yaml`) runs only the vda5050_connector pytest
suite in `ros:humble-ros-base`. To run the same thing, from the repo root:

```bash
.ci/docker/run.sh bash -c '
  sudo apt-get update -qq && rosdep update --rosdistro humble
  rosdep install -iy --rosdistro humble --from-paths \
    src/ros_amr_interop/vda5050_msgs src/ros_amr_interop/vda5050_connector
  colcon build --packages-up-to vda5050_connector
  source install/setup.bash && python3 -m pytest -q src/ros_amr_interop/vda5050_connector/test'
```

- The suite passes on `humble-devel` and takes seconds.
- `colcon test` also runs ament lint (clang_format, flake8, pep257). CI doesn't
  gate on these and they already fail on `humble-devel`. Compare against the
  base branch, and don't fix unrelated lint in a feature PR.
- The other CI gate is pre-commit: `pre-commit run --files <changed files>`.

## Pull requests

- CI doesn't run on PRs from forks until a maintainer approves the workflow
  run. No checks doesn't mean the checks passed.

## Releasing

- Releases go through bloom into `inorbit-ai/ros_amr_interop-release`.
  `rmf_inorbit_fleet_adapter` is not released for humble (`humble.ignored` on
  the release repo's `master`).
- `bloom-release` prompts need a real TTY. A human runs it in their own
  terminal, so the GitHub token never passes through the agent.
