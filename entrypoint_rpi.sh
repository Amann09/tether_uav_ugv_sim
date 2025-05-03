#!/bin/bash

# Start the MicroXRCEAgent
cd ~/Micro-XRCE-DDS-Agent/build && ./MicroXRCEAgent udp4 -p 8888 &

# Initialize ROS 2 environment and start relevant nodes
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

# Replace the simulation-specific ROS 2 nodes with real-life implementation nodes
# Example: ros2 run [package_name] [node_name] &
# Add your real-life ROS 2 nodes here

# Initialize and start custom scripts
# Assuming you have other scripts for real-life operation similar to the simulation
cd ~/scripts
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# Start any custom Python scripts for your application
# Example: python3 your_script.py &
# Replace or remove the following lines with your actual script names
python3 picam_publisher &
python3 aruco_detect.py &
python3 controller.py &


# Keep the script running to maintain the processes
while :; do sleep 1; done
