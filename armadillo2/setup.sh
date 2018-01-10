#!/bin/bash

# installation file for armadillo2 over ROS Kinetic and ubuntu 16.04 #

GREEN_TXT='\e[0;32m'
WHITE_TXT='\e[1;37m'
RED_TXT='\e[31m'
NO_COLOR='\033[0m'
LOGS_FOLDER='setup_logs'
LOGS_FOLDER_PATH="~/catkin_ws/src/${LOGS_FOLDER}"

printf "${WHITE_TXT}\n***Installing Armadillo2 ROS-Kinetic Package***\n${NO_COLOR}"
sleep 1

# validate ros version #
printf "${WHITE_TXT}\nChecking ROS Version...\n${NO_COLOR}"
sleep 1
version=`rosversion -d`
if [ "$version" == "kinetic" ]; then
  printf "${GREEN_TXT}ROS version OK${NO_COLOR}\n"
  sleep 1
else
  printf "${RED_TXT}Error: found ROS version ${version}, please install ROS Kinetic and try again${NO_COLOR}\n"
  exit 1
fi

# validate catkin_ws/src folder exist #
printf "${WHITE_TXT}\nChecking if catkin_ws/src folder exist...\n${NO_COLOR}"
sleep 1
cd ~ 
if [ ! -d "catkin_ws" ]; then
  printf "${RED_TXT}~/catkin_ws folder does not exist. Create workspace named catkin_ws and try again ${NO_COLOR}\n"
  exit 1
else
cd catkin_ws 
if [ ! -d "src" ]; then
  printf "${RED_TXT}~/catkin_ws/src folder does not exist. Create workspace named catkin_ws with src directory inside ${NO_COLOR}\n"
  exit 1
fi

fi
printf "${GREEN_TXT}found catkin_ws/src folder${NO_COLOR}\n"
sleep 1

# preparing error logs folder #
if [ ! -d ${LOGS_FOLDER} ]; then
  mkdir ${LOGS_FOLDER_PATH}
fi

# third party packages #
printf "${WHITE_TXT}Installing ROS and 3rd party packages...\n${NO_COLOR}"
{
sleep 1
# ROS packages #
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
# 3rd party packages #
sudo apt-get -y install jstest-gtk
printf "${GREEN_TXT}Done.\n\n${NO_COLOR}"
sleep 1
} 2> ${LOGS_FOLDER_PATH}/third_party_install_error.txt

# install xbox controller drivers #
printf "${WHITE_TXT}Installing xbox driver...\n${NO_COLOR}"
sleep 1
sudo apt-get -y install xboxdrv
sudo apt-get -y install sysfsutils
echo "module/bluetooth/parameters/disable_ertm = 1" >> /etc/sysfs.conf
printf "${GREEN_TXT}Done.\n\n${NO_COLOR}"
sleep 1

# install displaylink driver for mimo touch display #
printf "${WHITE_TXT}Installing displaylink driver...\n${NO_COLOR}"
sleep 1
sudo apt-get -y install linux-generic-lts-utopic xserver-xorg-lts-utopic 
sudo apt-get -y install libegl1-mesa-drivers-lts-utopic 
sudo apt-get -y install xserver-xorg-video-all-lts-utopic 
sudo apt-get -y install xserver-xorg-input-all-lts-utopic
sudo apt-get -y install dkms
cd ~/catkin_ws/src/armadillo2/armadillo2/third_party_files/
chmod +x displaylink-driver-4.1.9.run
sudo ./displaylink-driver-4.1.9.run
printf "${GREEN_TXT}Done.\n\n${NO_COLOR}"
sleep 1

# install softkinetic drivers #
printf "${WHITE_TXT}Installing softkinetic driver...\n${NO_COLOR}"
sleep 1
cd ~/catkin_ws/src/armadillo2/armadillo2/third_party_files/
sudo chmod +x ./DepthSenseSDK-1.9.0-5-amd64-deb.run
sudo ./DepthSenseSDK-1.9.0-5-amd64-deb.run
printf "${GREEN_TXT}Done.\n\n${NO_COLOR}"
sleep 1

# install kinect drivers #
printf "${WHITE_TXT}Installing kinect driver...\n${NO_COLOR}"
{
sleep 1
cd ~/catkin_ws/src/armadillo2/armadillo2_utils/libfreenect2
sudo apt-get -y install build-essential cmake pkg-config
sudo apt-get -y install libusb-1.0-0-dev
sudo apt-get -y install libturbojpeg libjpeg-turbo8-dev
sudo apt-get -y install libglfw3-dev
mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=$HOME/freenect2
make
make install
cd ~/catkin_ws/src/armadillo2/armadillo2_utils/iai_kinect2/iai_kinect2
rosdep install -r --from-paths .
printf "${GREEN_TXT}Done.\n\n${NO_COLOR}"
sleep 1
} 2> ${LOGS_FOLDER_PATH}/kinect_install_error.txt

# usb rules #
printf "${WHITE_TXT}Installing USB rules...\n${NO_COLOR}"
{
sleep 1
sudo apt -y install setserial #for setting port latency
sudo cp ~/catkin_ws/src/armadillo2/armadillo2/rules/usb_to_dxl.rules /etc/udev/rules.d
sudo cp ~/catkin_ws/src/armadillo2/armadillo2/rules/49-teensy.rules /etc/udev/rules.d
sudo cp ~/catkin_ws/src/armadillo2/armadillo2/rules/bms_battery.rules /etc/udev/rules.d
sudo cp ~/catkin_ws/src/armadillo2/armadillo2/rules/hokuyo.rules /etc/udev/rules.d/
sudo cp ~/catkin_ws/src/armadillo2/armadillo2_utils/libfreenect2/platform/linux/udev/90-kinect2.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && udevadm trigger
printf "${GREEN_TXT}Done.\n\n${NO_COLOR}"
sleep 1
} 2> ${LOGS_FOLDER_PATH}/usb_rules_error.txt

# compiling armadillo2#
printf "${WHITE_TXT}Compiling armadillo2 package...\n${NO_COLOR}"
{
sleep 1
cd ~/catkin_ws
catkin_make --pkg pr2_controllers_msgs robotican_msgs_srvs
catkin_make -DCMAKE_BUILD_TYPE="Release"
printf "${GREEN_TXT}Done.\n\n${NO_COLOR}"
sleep 1
} 2> ${LOGS_FOLDER_PATH}/catkin_make_error.txt

# remove empty log files #
for f in $LOGS_FOLDER_PATH
do
    if [[ !$(find $f -type f -size +0w 2>/dev/null) ]]; then
        rm $f
    fi
done

# if logs folder is not empty, some errors has occured"
cd $LOGS_FOLDER_PATH
if [ -z "$(ls)" ]; then
    printf "${GREEN_TXT}Installation process finished successfully.\n\n"  ${NO_COLOR}
    printf "${GREEN_TXT}Please reboot to apply changes\n\n${NO_COLOR}"
    rmdir $LOGS_FOLDER_PATH
    exit 0
else
    printf "${RED_TXT}Installation process finished, but some error occurred. please refer to ${LOGS_FOLDER_PATH} folder for more details.\n\n${NO_COLOR}"
    exit 1
fi


