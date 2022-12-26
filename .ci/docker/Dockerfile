ARG ROS_DISTRO="humble"

FROM ros:$ROS_DISTRO AS base

ENV DEBIAN_FRONTEND=noninteractive

# Install / update tools
RUN apt-get update \
   && apt-get install --no-install-recommends -y  \
   apt-transport-https \
   apt-utils \
   bash-completion \
   curl \
   git \
   libffi-dev \
   python3-colcon-common-extensions \
   python3-pip \
   ros-$ROS_DISTRO-ament-cmake-* \
   software-properties-common \
   sudo \
   tmux \
   unzip \
   vim \
   && rm -r /var/lib/apt/lists/*

# Add user `docker`, set password to `docker` and add it to group `sudo`
RUN useradd -m docker -s /bin/bash && echo "docker:docker" | chpasswd \
   && adduser docker sudo \
   && echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

USER docker

WORKDIR /home/docker/dev_ws

# Add ROS and workspace overlays to docker user
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/docker/.bashrc \
   && rosdep update
