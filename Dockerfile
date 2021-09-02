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

# Copy Website Files
WORKDIR /~
COPY uwrov_interface uwrov_interface