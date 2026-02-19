FROM ubuntu:22.04

WORKDIR /app  

COPY . .  

# Basic tools
RUN apt-get update && apt-get install -y \
    curl \
    gnupg \
    lsb-release

# Add ROS 2 GPG key
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add ROS 2 apt repository
RUN echo "deb [arch=$(dpkg --print-architecture) \
    signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu \
    $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/ros2.list

RUN apt-get update && apt-get install -y tzdata
ENV TZ=Europe/Athens
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

# Install ROS 2 Humble
RUN apt-get update && apt-get install -y \
    ros-humble-ros-base \
    ros-dev-tools

SHELL ["/bin/bash", "-c"]

RUN source /opt/ros/humble/setup.bash

# RUN alias ros2-build='colcon build --packages-select'
# RUN ros2-build gesture_recognition
# RUN source ./install/local_setup.zsh
# RUN apt install python3
# RUN pip install onnxruntime

CMD ["echo", "Hello from my Docker container!"]
