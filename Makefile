# Dirty Makefile. Depends on local environment.
# TODO: create docker image for building process.

WORKSPACE_DIR=~/dev_ws
SHELL := /bin/zsh  # Find a better way of doing this https://stackoverflow.com/a/43566158

clean:
	cd ${WORKSPACE_DIR}; \
	rm -rf build log install
	@echo "Workspace '${WORKSPACE_DIR}' cleaned"

build:
	@echo "Building package 'ros2-to-mass-amr-interop'"
	cd ${WORKSPACE_DIR}; \
	colcon build --packages-select ros2-to-mass-amr-interop

run:
	cd ${WORKSPACE_DIR}; \
	. install/setup.zsh; \
	ros2 run ros2_to_mass_amr_interop ros2_to_mass_node

tests:
	cd ${WORKSPACE_DIR}; \
	colcon test --packages-select ros2-to-mass-amr-interop; \
	colcon test-result --verbose

all: build run

pristine: clean build run