# Dirty Makefile. Depends on local environment.
# TODO: create docker image for building process.

build:
	@echo "Building package 'ross2mass'"
	cd ~/dev_ws/; \
	colcon build --packages-select ross2mass

clean:
	cd ~/dev_ws; \
	rm -rf build log install
	@echo "Workspace '~/dev_ws' cleaned"

all: clean build