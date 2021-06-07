# Dirty Makefile. Depends on local environment.
# TODO: create docker image for building process.

WORKSPACE_DIR=~/dev_ws
SHELL := /bin/zsh  # Find a better way of doing this https://stackoverflow.com/a/43566158

clean:
	cd ${WORKSPACE_DIR}; \
	rm -rf build log install
	@echo "Workspace '${WORKSPACE_DIR}' cleaned"

build:
	@echo "Building package 'ros2mass'"
	cd ${WORKSPACE_DIR}; \
	colcon build --packages-select ros2mass


run:
	@echo "WARNING/TODO: run target only supports zsh"
	cd ${WORKSPACE_DIR}; \
	. install/setup.zsh; \
	ros2 run ros2mass ros2mass


all: build run

pristine: clean build run