# syntax=docker/dockerfile:1
FROM ros:noetic-ros-core-focal

ENV ROS_DISTRO noetic

# Installing Dependencies
RUN apt-get update && apt-get install -y \
    git curl \
    build-essential \
    python3-pip \
    netbase

# Install python packages
COPY requirements.txt /requirements.txt
RUN pip3 install -r /requirements.txt

# Installing Node
ENV NODE_VERSION 14
RUN curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.35.3/install.sh | bash
RUN . ~/.bashrc \
  && nvm install $NODE_VERSION \
  && nvm alias default $NODE_VERSION \
  && nvm use default

# Create a catkin workspace
RUN mkdir -p /root/catkin_ws/src

# Copy Website Files
WORKDIR /root
COPY src/uwrov_interface /root/catkin_ws/src/uwrov_interface
COPY src/uwrov_server /root/catkin_ws/src/uwrov_server

# Install node dependencies
RUN . ~/.bashrc && . /opt/ros/${ROS_DISTRO}/setup.sh \
    && cd catkin_ws/src/uwrov_interface \
    && npm install

# Copy ROS Packages
COPY src/nautilus_launch /root/catkin_ws/src/nautilus_launch
COPY src/nautilus_scripts /root/catkin_ws/src/nautilus_scripts

# Build ROS Packages
RUN . ~/.bashrc && . /opt/ros/${ROS_DISTRO}/setup.sh \
    && cd catkin_ws \
    && catkin_make

RUN echo "source /root/catkin_ws/devel/setup.bash" >> ~/.bashrc

ENV ROS_LOG_DIR /root/logs

EXPOSE 3000