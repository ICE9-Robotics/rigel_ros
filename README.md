# Rigel
ROS package for the Rigel robot

# Dependencies
- Standard libraries
    - libasio
- ROS packages (hardware)
    - [scout_ros](https://github.com/agilexrobotics/scout_ros)
    - [ugv_sdk](https://github.com/agilexrobotics/ugv_sdk)
    - [velodyne](https://github.com/ros-drivers/velodyne)
    - [realsense-ros/ros1-legacy](https://github.com/IntelRealSense/realsense-ros/tree/ros1-legacy)
    - [Lslidar_ROS1_driver/C16_V4.0](https://github.com/Lslidar/Lslidar_ROS1_driver/tree/C16_V4.0)
    - [lslidar_description/ros](https://github.com/ICE9-Robotics/lslidar_description)
    - [microstrain_inertial](https://github.com/LORD-MicroStrain/microstrain_inertial)
- ROS packages (software)
    - [fast_lio](https://github.com/hku-mars/FAST_LIO/tree/main)

# Install
```
sudo apt install ros-$ROS_DISTRO-velodyne ros-$ROS_DISTRO-velodyne-description ros-$ROS_DISTRO-realsense2-camera ros-$ROS_DISTRO-realsense2-description ros-$ROS_DISTRO-microstrain-inertial-driver ros-$ROS_DISTRO-rosserial ros-$ROS_DISTRO-usb-cam
mkdir -p ~/rigel_ws/src
cd ~/rigel_ws/src

# ros pkg hardware
git clone git@github.com:ICE9-Robotics/rigel.git #private repo, requires ssh
git clone https://github.com/agilexrobotics/ugv_sdk.git
git clone https://github.com/agilexrobotics/scout_ros.git
git clone -b C16_V4.0 https://github.com/Lslidar/Lslidar_ROS1_driver.git
git clone -b ros https://github.com/ICE9-Robotics/lslidar_description.git
git clone https://github.com/AnthonyZJiang/scout_diagnostics.git

# fast_lio
git clone https://github.com/hku-mars/FAST_LIO.git --recurse-submodules

# install fast_lio dependency
cd ~
git clone https://github.com/Livox-SDK/livox_ros_driver.git ws_livox/src
cd ~/ws_livox
catkin_make
source devel/setup.sh

# make all packages
cd ~/rigel_ws
rosdep install --from-paths src -i
catkin_make

echo "source ~/rigel_ws/devel/setup.sh" >> .bashrc
```

Finally, set the IP Address of the ethernet interface of the onboard computer to 192.168.1.102

# Run
```
roslaunch rigel_ros rigel.launch rviz:=true
```

# 3D reconstruction

# To view 3D reconstruction
```
pcl_viewer ~/rigel_ws/src/FAST_LIO/PCD/scans.pcd -ps 3
```
Press 1-5 to change the colour scheme.