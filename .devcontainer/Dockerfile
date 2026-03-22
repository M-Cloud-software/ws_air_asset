# This is what Robbie gave
# FROM mcr.microsoft.com/devcontainers/base:jammy

# This is what Claude recommended since the jetson is an ARM64 image
FROM mcr.microsoft.com/devcontainers/base:ubuntu-22.04

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

RUN apt-get update && apt-get install -y \
    sudo \
    wget \
    curl \
    git \
    unzip \
    zip \
    ca-certificates \
    gnupg \
    lsb-release \
    software-properties-common \
    locales \
    build-essential \
    cmake \
    ninja-build \
    pkg-config \
    ccache \
    file \
    xz-utils \
    python3-pip \
    python3-venv \
    python3-setuptools \
    python3-wheel \
    python3-jinja2 \
    python3-empy \
    python3-toml \
    python3-numpy \
    python3-yaml \
    python3-dev \
    python3-gi \
    python3-gst-1.0 \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-libav \
    gstreamer1.0-gl \
    libfuse2 \
    libxcb-xinerama0 \
    libxkbcommon-x11-0 \
    libxcb-cursor-dev \
    libgl1 \
    libegl1 \
    libx11-xcb1 \
    libxrender1 \
    libxtst6 \
    libxi6 \
    libxcomposite1 \
    libxcursor1 \
    libxdamage1 \
    libxrandr2 \
    libasound2 \
    libgtk-3-0 \
    libnss3 \
    libdrm2 \
    mesa-utils \
    libpipewire-0.3-0 \
    pipewire \
    speech-dispatcher \
    && rm -rf /var/lib/apt/lists/*

RUN locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# ROS 2 Humble repo
RUN add-apt-repository universe -y && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
      > /etc/apt/sources.list.d/ros2.list

# ROS packages only
RUN apt-get update && apt-get install -y \
    ros-humble-desktop \
    ros-dev-tools \
    ros-humble-vision-msgs \
    ros-humble-image-transport \
    ros-humble-cv-bridge \
    && rm -rf /var/lib/apt/lists/*



# PX4 setup script handles Gazebo Harmonic on Ubuntu 22.04
# RUN git clone --depth 1 https://github.com/PX4/PX4-Autopilot.git /opt/PX4-Autopilot && \
#     /bin/bash /opt/PX4-Autopilot/Tools/setup/ubuntu.sh || true

# Python extras in venv
RUN python3 -m venv /opt/px4-venv && \
    /opt/px4-venv/bin/pip install --upgrade pip setuptools wheel && \
    /opt/px4-venv/bin/pip install empy==3.3.4 pyros-genmsg lark catkin_pkg opencv-python-headless

# Micro XRCE-DDS Agent
WORKDIR /opt
RUN git clone -b v2.4.3 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git && \
    cd Micro-XRCE-DDS-Agent && \
    mkdir -p build && \
    cd build && \
    cmake .. && \
    make -j"$(nproc)" && \
    make install && \
    ldconfig /usr/local/lib

WORKDIR /opt/qgroundcontrol

RUN echo 'source /opt/ros/humble/setup.bash' >> /etc/bash.bashrc && \
    # echo 'source /opt/px4-venv/bin/activate' >> /etc/bash.bashrc && \
    echo 'alias px4env="source /opt/px4-venv/bin/activate"' >> /etc/bash.bashrc && \ 
    echo 'alias qgc="QT_QPA_PLATFORM=xcb /opt/qgroundcontrol/qgc/AppRun"' >> /etc/bash.bashrc && \
    echo 'alias gzsim="gz sim"' >> /etc/bash.bashrc

WORKDIR /workspaces
