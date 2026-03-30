# ROS2 Humble + Gazebo Harmonic Docker Image
# For educational purposes - ROS2 Course (Windows)
FROM ros:humble-ros-base-jammy

# Prevent interactive prompts during installation
ENV DEBIAN_FRONTEND=noninteractive

# Set up locale
RUN apt-get update && apt-get install -y locales && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8

# Install essential tools and dependencies
RUN apt-get update && apt-get install -y \
    curl \
    gnupg \
    lsb-release \
    wget \
    git \
    vim \
    nano \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    build-essential \
    cmake \
    sudo \
    x11-apps \
    mesa-utils \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    && rm -rf /var/lib/apt/lists/*

# Add Gazebo Harmonic repository
RUN curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# Install Gazebo Harmonic
RUN apt-get update && apt-get install -y \
    gz-harmonic \
    && rm -rf /var/lib/apt/lists/*

# Install ROS2-Gazebo integration packages
RUN apt-get update && apt-get install -y \
    ros-humble-ros-gz \
    ros-humble-rviz2 \
    ros-humble-rqt* \
    ros-humble-teleop-twist-keyboard \
    ros-humble-xacro \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-gz-ros2-control \
    ros-humble-joy \
    ros-humble-joy-teleop \
    ros-humble-robot-localization \
    ros-humble-turtlesim \
    ros-humble-turtle-tf2-py \
    ros-humble-tf2-tools \
    ros-humble-slam-toolbox \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    && rm -rf /var/lib/apt/lists/*

# Install system and Python dependencies
RUN apt-get update && apt-get install -y \
    libserial-dev \
    python3-serial \
    python3-smbus \
    ros-humble-tf-transformations \
    ros-humble-foxglove-bridge \
    && rm -rf /var/lib/apt/lists/*

# Create a non-root user for the students
ARG USERNAME=ros2user
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME && \
    useradd --uid $USER_UID --gid $USER_GID -m $USERNAME && \
    echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME

# Create workspace directory
RUN mkdir -p /home/$USERNAME/ros2_ws/src && \
    chown -R $USERNAME:$USERNAME /home/$USERNAME/ros2_ws

# Set up ROS2 environment in bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> /home/$USERNAME/.bashrc && \
    echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> /home/$USERNAME/.bashrc && \
    echo "export GZ_SIM_RESOURCE_PATH=\$GZ_SIM_RESOURCE_PATH:/home/$USERNAME/ros2_ws/src" >> /home/$USERNAME/.bashrc && \
    echo "" >> /home/$USERNAME/.bashrc && \
    echo "# Helpful aliases" >> /home/$USERNAME/.bashrc && \
    echo "alias cb='cd ~/ros2_ws && colcon build --symlink-install'" >> /home/$USERNAME/.bashrc && \
    echo "alias sw='source ~/ros2_ws/install/setup.bash'" >> /home/$USERNAME/.bashrc && \
    echo "alias cbsw='cb && sw'" >> /home/$USERNAME/.bashrc

# Switch to non-root user
USER $USERNAME
WORKDIR /home/$USERNAME/ros2_ws

# Set entrypoint
ENTRYPOINT ["/bin/bash"]
