FROM ros:iron

ENV TZ="Europe/Berlin"
RUN date

# Set non-interactive mode to avoid prompts during installation
ENV DEBIAN_FRONTEND=noninteractive

RUN bash -c "cd /home && mkdir ros2_ws && cd ros2_ws && mkdir config"

COPY ./config /home/ros2_ws/config

# Preconfigure debconf selections for keyboard-configuration package
RUN echo 'keyboard-configuration keyboard-configuration/xkb-keymap select us' | debconf-set-selections && \
    echo 'keyboard-configuration keyboard-configuration/layout select 43' | debconf-set-selections && \
    echo 'keyboard-configuration keyboard-configuration/model select 6' | debconf-set-selections

# Update and install necessary packages in one step to reduce image layers
RUN apt-get update && apt-get install -y \
    ros-iron-navigation2 \
    ros-iron-nav2-bringup \
    ros-iron-turtlebot3* \
    ros-iron-rviz2 \
    nano \
    && rm -rf /var/lib/apt/lists/*

# Set default environment variables for TurtleBot3
ENV TURTLEBOT3_MODEL=waffle
ENV GAZEBO_MODEL_PATH='/opt/ros/iron/share/turtlebot3_gazebo/models:$GAZEBO_MODEL_PATH'

# Set default values for TurtleBot3 simulation (can be adjusted as per use case)
ENV TURTLEBOT3_DESCRIPTION='/opt/ros/iron/share/turtlebot3_description'
ENV TURTLEBOT3_GAZEBO_WORLD='/opt/ros/iron/share/turtlebot3_gazebo/worlds/turtlebot3_worlds.world'

# Source ROS 2 environment and set TurtleBot3 model
RUN echo "source /opt/ros/iron/setup.bash" >> ~/.bashrc

ENV ROS_DOMAIN_ID=0

# Reset DEBIAN_FRONTEND after installation
ENV DEBIAN_FRONTEND=

COPY ./entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/bin/bash"]