#!/bin/bash

# installation file for armadillo2 over ROS Kinetic and ubuntu 16.04 #

GREEN_TXT='\e[0;32m'
WHITE_TXT='\e[1;37m'
RED_TXT='\e[31m'
NO_COLOR='\033[0m'
LOGS_FOLDER_PATH="${HOME}/catkin_ws/src/setup_logs"

printf "${WHITE_TXT}\n***Installing Armadillo2 ROS-Kinetic Package***\n${NO_COLOR}"

# validate ros version #
printf "${WHITE_TXT}\nChecking ROS Version...\n${NO_COLOR}"

version=`rosversion -d`
if [ "$version" == "kinetic" ]; then
  printf "${GREEN_TXT}ROS version OK${NO_COLOR}\n"
else
  printf "${RED_TXT}Error: found ROS version ${version}, please install ROS Kinetic and try again${NO_COLOR}\n"
  exit 1
fi

# validate catkin_ws/src folder exist #
printf "${WHITE_TXT}\nChecking if catkin_ws/src folder exist...\n${NO_COLOR}"
cd ~ 
if [ ! -d "catkin_ws" ]; then
  printf "${RED_TXT}~/catkin_ws folder does not exist. Create workspace named catkin_ws and try again ${NO_COLOR}\n"
else
cd catkin_ws 
if [ ! -d "src" ]; then
  printf "${RED_TXT}~/catkin_ws/src folder does not exist. Create workspace named catkin_ws with src directory inside ${NO_COLOR}\n"
  exit 1
fi

fi
printf "${GREEN_TXT}found catkin_ws/src folder${NO_COLOR}\n"

# third party packages #
sudo apt-get update
sudo apt-get dist-upgrade 
sudo apt-get upgrade 
sudo apt-get -y install ros-kinetic-controller-manager 
sudo apt-get -y install ros-kinetic-control-toolbox  
sudo apt-get -y install ros-kinetic-transmission-interface 
sudo apt-get -y install ros-kinetic-joint-limits-interface 
sudo apt-get -y install ros-kinetic-gazebo-ros-control 
sudo apt-get -y install ros-kinetic-ros-controllers 
sudo apt-get -y install ros-kinetic-ros-control 
sudo apt-get -y install ros-kinetic-moveit
sudo apt-get -y install ros-kinetic-moveit-ros-planning
sudo apt-get -y install ros-kinetic-moveit-ros-planning-interface
sudo apt-get -y install ros-kinetic-move-base
sudo apt-get -y install ros-kinetic-navigation
sudo apt-get -y install ros-kinetic-hector-slam
sudo apt-get -y install ros-kinetic-gmapping
sudo apt-get -y install ros-kinetic-twist-mux
sudo apt-get -y install ros-kinetic-pid
sudo apt-get -y install ros-kinetic-joy
sudo apt-get -y install ros-kinetic-ar-track-alvar
sudo apt-get -y install joystick
sudo apt-get -y install ros-kinetic-hector-gazebo-plugins
sudo apt-get -y install ros-kinetic-serial
sudo apt-get -y install espeak espeak-data libespeak-dev
sudo apt-get -y install ros-kinetic-robot-localization
sudo apt-get -y install ros-kinetic-trac-ik ros-kinetic-moveit-kinematics 
sudo apt-get -y install ros-kinetic-urg-node 
sudo apt-get -y install ros-kinetic-rtabmap-ros
sudo apt-get -y install jstest-gtk
printf "${GREEN_TXT}Done.\n\n${NO_COLOR}"

# install xbox controller drivers #
sudo apt-get -y install xboxdrv
sudo apt-get -y install sysfsutils
sudo /bin/su -c "echo 'module/bluetooth/parameters/disable_ertm = 1' >> /etc/sysfs.conf"
printf "${GREEN_TXT}Done.\n\n${NO_COLOR}"

# install displaylink driver for mimo touch display, #
# and place xprofile script to enforce resulotion    #
sudo apt-get -y install linux-generic-lts-utopic xserver-xorg-lts-utopic 
sudo apt-get -y install libegl1-mesa-drivers-lts-utopic 
sudo apt-get -y install xserver-xorg-video-all-lts-utopic 
sudo apt-get -y install xserver-xorg-input-all-lts-utopic
sudo apt-get -y install dkms
cd ~/catkin_ws/src/armadillo2/armadillo2/third_party_files/
chmod +x displaylink-driver-4.1.9.run
sudo ./displaylink-driver-4.1.9.run
cp .xprofile ~/
printf "${GREEN_TXT}Done.\n\n${NO_COLOR}"

# install softkinetic drivers #
cd ~/catkin_ws/src/armadillo2/armadillo2/third_party_files/
sudo chmod +x ./DepthSenseSDK-1.9.0-5-amd64-deb.run
sudo ./DepthSenseSDK-1.9.0-5-amd64-deb.run

# install kinect drivers #
sudo apt-get -y install build-essential cmake pkg-config
sudo apt-get -y install libusb-1.0-0-dev
sudo apt-get -y install libturbojpeg libjpeg-turbo8-dev
sudo apt-get -y install libglfw3-dev
cd ~/catkin_ws/src/armadillo2/armadillo2_utils/libfreenect2
mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=$HOME/freenect2
make
sudo make install
cd ~/catkin_ws/src/armadillo2/armadillo2_utils/iai_kinect2/iai_kinect2
rosdep install -r --from-paths .
printf "${GREEN_TXT}Done.\n\n${NO_COLOR}"

# usb rules #
sudo apt -y install setserial #for setting port latency
sudo cp ~/catkin_ws/src/armadillo2/armadillo2/rules/usb_to_dxl.rules /etc/udev/rules.d
sudo cp ~/catkin_ws/src/armadillo2/armadillo2/rules/roboteq.rules /etc/udev/rules.d
sudo cp ~/catkin_ws/src/armadillo2/armadillo2/rules/49-teensy.rules /etc/udev/rules.d
sudo cp ~/catkin_ws/src/armadillo2/armadillo2/rules/bms_battery.rules /etc/udev/rules.d
sudo cp ~/catkin_ws/src/armadillo2/armadillo2/rules/hokuyo.rules /etc/udev/rules.d/
sudo cp ~/catkin_ws/src/armadillo2/armadillo2_utils/libfreenect2/platform/linux/udev/90-kinect2.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && udevadm trigger
printf "${GREEN_TXT}Done.\n\n${NO_COLOR}"

# compiling armadillo2 #
printf "${WHITE_TXT}Compiling armadillo2 package...\n${NO_COLOR}"
cd ~/catkin_ws
catkin_make -DCMAKE_BUILD_TYPE="Release"
printf "${GREEN_TXT}Done.\n\n${NO_COLOR}"

printf "${GREEN_TXT}Installation process finished.\n\n"  ${NO_COLOR}
printf "${GREEN_TXT}Please reboot to apply changes\n\n${NO_COLOR}"
exit 0
