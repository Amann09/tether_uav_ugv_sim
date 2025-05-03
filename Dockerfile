# Use an NVIDIA compatible base image
FROM osrf/ros:humble-desktop-full


ENV LANG C.UTF-8

RUN echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections

RUN apt-get update && apt-get install -y \
    locales \
    curl \
    software-properties-common \
    python3-venv \
    git \
    cmake \
    build-essential \
    python3-pip \
    libopencv-dev \
    ros-humble-desktop \
    ros-humble-navigation2 \ 
    ros-humble-nav2-bringup \
    gedit \
    xterm \ 
    ros-dev-tools \
    && rm -rf /var/lib/apt/lists/*

# Set the nvidia container runtime
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

RUN sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
RUN apt-get update && apt-get upgrade -y


# Set up locales
RUN locale-gen en_US en_US.UTF-8
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG en_US.UTF-8


# Source ROS 2 setup.bash in .bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Install additional Python packages
RUN pip3 install --user -U empy==3.3.4 pyros-genmsg setuptools==59.6.0 mavsdk aioconsole pygame numpy==1.26.4 opencv-contrib-python tabulate networkx ortools

# Clone the PX4-Autopilot repository and build PX4
WORKDIR /root
RUN git clone https://github.com/PX4/PX4-Autopilot.git --recursive -b release/1.15
RUN bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
WORKDIR /root/PX4-Autopilot
RUN make px4_sitl

# Clone Micro XRCE-DDS Agent and build it
WORKDIR /root
RUN git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
WORKDIR /root/Micro-XRCE-DDS-Agent/build
RUN cmake ..
RUN make
RUN make install
RUN ldconfig /usr/local/lib/

# Clone and build ROS 2 workspaces
WORKDIR /root
RUN mkdir -p ~/ros2_ws/src/
RUN git clone https://github.com/PX4/px4_msgs.git  /root/ros2_ws/src/px4_msgs
RUN git clone https://github.com/PX4/px4_ros_com.git /root/ros2_ws/src/px4_ros_com
ENV GZ_VERSION=garden
RUN git clone https://github.com/gazebosim/ros_gz.git -b humble /root/ros2_ws/src/ros_gz

RUN git clone https://github.com/Amann09/ros_gz_tugbot.git /root/ros2_ws/src/ros_gz_tugbot

# RUN git clone https://github.com/clearpathrobotics/clearpath_simulator.git /root/ros2_ws/src/clearpath_simulator
# RUN /bin/bash -c "mkdir ~/clearpath"
# COPY  scripts/robot.yaml /root/clearpath/robot.yaml

RUN /bin/bash -c "cd ~/ros2_ws/ && mkdir yaml_files"
COPY  scripts/bridge.yaml /root/ros2_ws/yaml_files/bridge.yaml

# RUN git clone https://github.com/clearpathrobotics/clearpath_common.git -b humble /root/ros2_ws/src/clearpath_common
# RUN git clone https://github.com/clearpathrobotics/clearpath_config.git /root/ros2_ws/src/clearpath_config
# RUN git clone https://github.com/clearpathrobotics/clearpath_msgs.git /root/ros2_ws/src/clearpath_msgs

# RUN sudo apt install python3-vcstool
# RUN /bin/bash -c "source /opt/ros/humble/setup.bash && cd ~/ros2_ws/ && wget https://raw.githubusercontent.com/clearpathrobotics/clearpath_simulator/main/dependencies.repos && vcs import src < dependencies.repos"
# RUN git clone https://github.com/clearpathrobotics/clearpath_mecanum_drive_controller.git /root/ros2_ws/src/clearpath_mecanum_drive_controller

RUN /bin/bash -c "cd /root/ros2_ws && rosdep install -r --from-paths src -i -y --rosdistro humble"

RUN /bin/bash -c "source /opt/ros/humble/setup.bash && cd ~/ros2_ws/ && colcon build --symlink-install "
RUN echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
RUN echo "export GZ_SIM_RESOURCE_PATH=~/.gz/models" >> ~/.bashrc

# Getting my code
COPY uav-ugv-path-planning /root/uav-ugv-path-planning

RUN rm -rf /root/PX4-Autopilot/Tools/simulation/gz/worlds /root/PX4-Autopilot/Tools/simulation/gz/models/x500_depth
COPY  x500_depth /root/PX4-Autopilot/Tools/simulation/gz/models/x500_depth
COPY  worlds /root/PX4-Autopilot/Tools/simulation/gz/worlds
# COPY  assets/7x7_1000-29.jpg /root/PX4-Autopilot/Tools/simulation/gz/worlds/materials/textures/7x7_1000-29.jpg
COPY  .gz /root/.gz
COPY  entrypoint.sh /root/entrypoint.sh
RUN  chmod +x /root/entrypoint.sh

ENTRYPOINT ["/root/entrypoint.sh"]