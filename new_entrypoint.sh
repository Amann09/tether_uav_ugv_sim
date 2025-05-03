 #!/bin/bash
# Terminal #1
cd ~/Micro-XRCE-DDS-Agent && MicroXRCEAgent udp4 -p 8888 &


cd ~/ros2_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 run ros_gz_image image_bridge /camera &

# Terminal #2
xterm -e "cd ~/PX4-Autopilot && PX4_SYS_AUTOSTART=4002  PX4_GZ_MODEL=x500_depth ./build/px4_sitl_default/bin/px4;/bin/bash" &

# Terminal #4
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash && cd ~/scripts &&  python3 aruco_detect.py &

# Terminal #5
xterm -e "source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash && sleep 12 && cd ~/scripts && python3 controller.py --takeoff & exec bash" &

xterm -e "source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash ; /bin/bash" &

# Keep the script running to keep the terminals open
while :; do sleep 1; done