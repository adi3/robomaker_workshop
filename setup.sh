#!/usr/bin/env bash

sudo apt update && sudo apt upgrade -y

sudo apt install python-pip fswebcam awscli -y
pip install boto3

#aws configure set profile.robomaker_workshop.role_arn arn:aws:iam::517502204741:role/ResourcesForRoboticsWorkshop
#aws configure set profile.robomaker_workshop.source_profile default
#aws configure set profile.robomaker_workshop.region eu-central-1

mkdir -p ~/aws_ws/src/ && cd ~/aws_ws/src/
git clone -b completed https://github.com/adi3/robomaker_workshop

# Pull dependencies
cd ~/
curl https://raw.githubusercontent.com/Interbotix/interbotix_ros_manipulators/main/interbotix_ros_xsarms/install/rpi4/xsarm_rpi4_install.sh > xsarm_rpi4_install.sh
chmod +x xsarm_rpi4_install.sh
./xsarm_rpi4_install.sh -n -p ~/aws_ws
rm xsarm_rpi4_install.sh
mv ~/interbotix_ws/src/* ~/aws_ws/src/
rm -rf ~/interbotix_ws

# Temporary fix to last upstream working version
cd ~/aws_ws/src/interbotix_ros_toolboxes
git checkout 1c8652021575e8980eb592aaa13ad294147329a0
cd -

#echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
echo "source ~/aws_ws/devel/setup.bash" >> ~/.bashrc
#echo "export ROS_PACKAGE_PATH=/home/ubuntu/interbotix_ws/src:$ROS_PACKAGE_PATH" >> ~/.bashrc
#echo "export PYTHONPATH=/home/ubuntu/interbotix_ws/devel/lib/python2.7/dist-packages:$PYTHONPATH" >> ~/.bashrc

# Build workspace
cd ~/aws_ws
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro melodic -r -y
source /opt/ros/melodic/setup.bash
catkin_make

echo "[[ -f ~/.bashrc ]] && source ~/.bashrc" >> ~/.bash_profile
source ~/.bash_profile
sudo reboot
