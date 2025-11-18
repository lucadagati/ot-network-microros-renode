FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# ----------------------------
# 1. Base system
# ----------------------------
RUN apt-get update && apt-get install -y \
    locales curl wget git \
    build-essential cmake \
    python3 python3-pip python3-venv \
    gcc-arm-none-eabi gdb-multiarch \
    gcc-riscv64-linux-gnu \
    ## fix dependency missing in rosdep (flex)
    flex bison \
    ## additional dependencies often missing
    libpython3-dev python3-empy python3-setuptools python3-yaml \
    ## Renode dependencies
    libgtk-3-0 libnotify4 libnss3 libxss1 libxtst6 xdg-utils \
    libatspi2.0-0 libappindicator3-1 libsecret-1-0 \
    mono-complete \
    && locale-gen en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*

# ----------------------------
# 2. Install rosdep, vcstool, colcon via pip
# ----------------------------
RUN pip3 install \
    rosdep \
    vcstool \
    colcon-common-extensions

# ----------------------------
# 3. Init rosdep
# ----------------------------
RUN rosdep init || true

# ----------------------------
# 4. Install ROS 2 Humble FIRST (required for micro-ROS build)
# ----------------------------
RUN apt-get update && apt-get install -y \
    software-properties-common \
    && add-apt-repository universe \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - \
    && add-apt-repository "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
    && apt-get update && apt-get install -y \
    ros-humble-desktop \
    ros-humble-fastrtps \
    && rm -rf /var/lib/apt/lists/*

# Setup ROS 2 environment for subsequent builds
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

# ----------------------------
# 5. Create workspace and clone micro-ROS setup
# ----------------------------
RUN mkdir -p /microros_ws/src
WORKDIR /microros_ws/src

RUN git clone -b humble https://github.com/micro-ROS/micro_ros_setup.git micro_ros_setup

# ----------------------------
# 6. Install micro-ROS dependencies
# ----------------------------
WORKDIR /microros_ws
RUN rosdep update && \
    rosdep install --from-paths src --ignore-src -y --rosdistro humble --skip-keys="microxrcedds ament_lint_auto ament_lint_common" || echo "Some rosdep packages skipped, continuing..."

# ----------------------------
# 7. Build micro-ROS setup tools (now ROS 2 is available)
# ----------------------------
RUN bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install"

# ----------------------------
# 8. Install Renode (portable version for headless/SSH use)
# ----------------------------
# Reference: https://renode.io/news/test-driven-development-of-zephyr%2Bmicro-ros-with-renode/
RUN mkdir -p /opt/renode && cd /opt/renode && \
    wget -q https://builds.renode.io/renode-latest.linux-portable.tar.gz && \
    tar -xzf renode-latest.linux-portable.tar.gz && \
    mv renode_* renode_portable && \
    rm renode-latest.linux-portable.tar.gz && \
    ln -s /opt/renode/renode_portable/renode /usr/local/bin/renode && \
    chmod +x /usr/local/bin/renode

# ----------------------------
# 9. Setup workspace and scripts
# ----------------------------
WORKDIR /workspace
RUN mkdir -p /workspace

# ----------------------------
# 10. Install tmux for SSH session management
# ----------------------------
RUN apt-get update && apt-get install -y tmux screen && \
    rm -rf /var/lib/apt/lists/*

# ----------------------------
# 11. Default environment
# ----------------------------
RUN echo "source /microros_ws/install/setup.bash" >> /root/.bashrc && \
    echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "export PATH=/opt/renode/renode_portable:\$PATH" >> /root/.bashrc && \
    echo "cd /workspace" >> /root/.bashrc

WORKDIR /workspace
CMD ["/bin/bash"]
