#!/bin/bash

GREEN_TXT='\e[0;32m'
WHITE_TXT='\e[1;37m'
RED_TXT='\e[31m'
NO_COLOR='\033[0m'

printf "${WHITE_TXT}\n***Installing Armadillo2 ROS-Kinetic Package***\n${NO_COLOR}"

# validate ros version #
printf "${WHITE_TXT}\nChecking ROS Version...\n${NO_COLOR}"
version=`rosversion -d`
if [ "$version" == "kinetic" ]; then
  printf "${GREEN_TXT}ROS version OK${NO_COLOR}\n"
else
  printf "${RED_TXT}Error: Ros version is not kinetic, please install ros kinetic and try again${NO_COLOR}\n"
  exit 1
fi

# third party packages #
printf "${WHITE_TXT}Installing dependencies and 3rd party packages...\n${NO_COLOR}"

sudo apt-get update 
sudo apt-get -y install ros-kinetic-controller-manager 
sudo apt-get -y install ros-kinetic-control-toolbox  
sudo apt-get -y install ros-kinetic-transmission-interface 
sudo apt-get -y install ros-kinetic-joint-limits-interface 
sudo apt-get -y install ros-kinetic-gazebo-ros-control 
sudo apt-get -y install ros-kinetic-ros-controllers 
sudo apt-get -y install ros-kinetic-ros-control 
sudo apt-get install ros-kinetic-moveit
sudo apt-get install ros-kinetic-pid
sudo apt-get install ros-kinetic-joy
sudo apt-get install joystick 

# install softkinetic drivers #
cd ~/catkin_ws/src/armadillo2/armadillo2/third_party_files/
sudo chmod +x ./DepthSenseSDK-1.9.0-5-amd64-deb.run
sudo ./DepthSenseSDK-1.9.0-5-amd64-deb.run
printf "${GREEN_TXT}Done.\n\n${NO_COLOR}"

# usb rules #
printf "${WHITE_TXT}Installing USB rules...\n${NO_COLOR}"
sudo apt -y install setserial #for setting port latency
sudo cp ~/catkin_ws/src/armadillo2/armadillo2/rules/usb_to_dxl.rules /etc/udev/rules.d
sudo cp ~/catkin_ws/src/armadillo2/armadillo2/rules/49-teensy.rules /etc/udev/rules.d
sudo cp ~/catkin_ws/src/armadillo2/armadillo2/rules/bms_battery.rules /etc/udev/rules.d
sudo udevadm control --reload-rules && udevadm trigger

printf "${GREEN_TXT}Done.\n\n${NO_COLOR}"

# compiling #
printf "${WHITE_TXT}Compiling...\n${NO_COLOR}"

cd ../../.. && catkin_make

printf "${GREEN_TXT}Done.\n\n${NO_COLOR}"


exit 0