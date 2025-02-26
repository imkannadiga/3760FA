FROM ros:iron

# Set timezone and non-interactive mode
ENV TZ="Europe/Berlin"
ENV DEBIAN_FRONTEND=noninteractive

# Create workspace directory
WORKDIR /home/ros2_ws

# Copy source code into container
COPY . .

# Install required packages
RUN apt-get update && \
    apt-get install -y python3-pip && \
    pip install websockets requests python-dotenv

# Install ROS 2 dependencies
RUN apt-get install -y ros-iron-nav2-msgs ros-iron-geometry-msgs ros-iron-rclpy

# Build ROS 2 workspace
RUN colcon build

# Set environment variables
ENV ROS_DOMAIN_ID=0

# Reset DEBIAN_FRONTEND after installation
ENV DEBIAN_FRONTEND=

# Use an entrypoint script instead of sourcing directly
COPY ./entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]