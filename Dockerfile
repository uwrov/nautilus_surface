FROM ros:melodic-ros-core-bionic

# Installing Dependencies
RUN apt-get update && apt-get install -y \
    git curl

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
COPY uwrov_interface /root/catkin_ws/src/uwrov_interface
COPY uwrov_server /root/catkin_ws/src/uwrov_server
COPY nautilus_launch /root/catkin_ws/src/nautilus_launch
COPY nautilus_scripts /root/catkin_ws/src/nautilus_scripts

# Install node dependencies
RUN . ~/.bashrc && . /opt/ros/melodic/setup.sh \
    && cd catkin_ws/src/uwrov_interface \
    && npm install