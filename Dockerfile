# Use the official ROS Humble base image for ARM64
FROM arm64v8/ros:humble-ros-base

# Set the working directory
WORKDIR /root/docker

# Copy the package source files into the container
COPY . .

# Install necessary dependencies and tools, including GPIO libraries for Raspberry Pi
RUN apt-get update && apt-get install -y \
    sudo \
    python3-colcon-common-extensions \
    ros-humble-desktop \
    ros-humble-nav2* \
    ros-humble-twist-mux \
    ros-humble-rviz2 \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-robot-state-publisher \
    ros-humble-xacro \
    ros-humble-robot-localization \
    ros-humble-tf2-ros \
    ros-humble-tf2-tools \
    ros-humble-cartographer \
    ros-humble-slam-toolbox \
    ros-humble-cartographer-ros \
    ros-humble-rplidar-ros \
    ros-humble-joy-linux \
    ros-humble-laser-filters \
    ros-humble-serial-driver \
    ros-humble-ament-index-cpp \
    ros-humble-geometry-msgs \
    ros-humble-sensor-msgs \
    ros-humble-std-msgs \
    ros-humble-std-srvs \
    libceres-dev \
    nano \
    wget \
    gnupg2 \
    apt-transport-https \
    software-properties-common \
    python3-rpi.gpio \
    python3-pip \
    && rm -rf /var/lib/apt/lists/* || true

# Install necessary Python packages via pip, even if they fail
RUN sudo pip install RPi.GPIO || true

# Initialize and update rosdep
RUN sudo rm -rf /etc/ros/rosdep/sources.list.d/20-default.list && \
    sudo rosdep init && \
    rosdep update || true

# Install dependencies using rosdep
RUN rosdep install --from-paths src --ignore-src -r -y || true

# Build the workspace, continue even if it fails
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build" || true

# Source the setup script
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
RUN echo "source /root/docker/install/setup.bash" >> /root/.bashrc

# Set the entrypoint to bash
ENTRYPOINT ["/bin/bash"]
