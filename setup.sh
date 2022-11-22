#!/usr/bin/env bash

export DISPLAY=:0

########################################################################################
#------------------------------------ Begin setup -------------------------------------#
########################################################################################
# aws configure set profile.robomaker_workshop.role_arn arn:aws:iam::007281172979:role/ResourcesForRoboticsWorkshop
# aws configure set profile.robomaker_workshop.source_profile default
# aws configure set profile.robomaker_workshop.region us-east-2

########################################################################################
#------------------------------------- End setup --------------------------------------#
########################################################################################

# Pull code from workshop studio
mkdir -p /home/ubuntu/environment/aws_ws/src/
cd /home/ubuntu/environment/aws_ws/src/
unzip /home/ubuntu/robomaker_workshop.zip

cd /home/ubuntu/environment/aws_ws
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro melodic -r -y

# Pull dependencies
cd /home/ubuntu/environment
curl "https://raw.githubusercontent.com/Interbotix/interbotix_ros_manipulators/main/interbotix_ros_xsarms/install/amd64/xsarm_amd64_install.sh" > xsarm_amd64_install.sh
chmod +x xsarm_amd64_install.sh
./xsarm_amd64_install.sh -d melodic -n
rm xsarm_amd64_install.sh

# Temporary fix to last upstream working version
cd /home/ubuntu/environment/aws_ws/src/interbotix_ros_toolboxes
git checkout 1c8652021575e8980eb592aaa13ad294147329a0
cd -

#apt install python3-catkin-tools python3-catkin-pkg imagemagick -y
apt install python-catkin-tools imagemagick -y
# pip install boto3
# pip2 install boto3

mv /home/ubuntu/interbotix_ws/src/* /home/ubuntu/environment/aws_ws/src/
#rm -rf /home/ubuntu/interbotix_ws/src
cd /home/ubuntu/environment/aws_ws
#rosdep fix-permissions
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro melodic -r -y

# Build project
source /opt/ros/melodic/setup.bash
#catkin_make
catkin_make

# sudo su -l ubuntu -c 'echo "export DISPLAY=:0" >> ~/.bashrc'
# sudo su -l ubuntu -c 'echo "source /home/ubuntu/environment/aws_ws/devel/setup.bash" >> ~/.bashrc'
# sudo su -l ubuntu -c 'echo "[[ -f ~/.bashrc ]] && source ~/.bashrc" >> ~/.bash_profile'
echo "export DISPLAY=:0" >> ~/.bashrc
echo "source /home/ubuntu/environment/aws_ws/devel/setup.bash" >> ~/.bashrc
echo "[[ -f ~/.bashrc ]] && source ~/.bashrc" >> ~/.bash_profile

# Write Ignition config file to prevent soft Gazebo error
sudo -u ubuntu cat > /home/ubuntu/.ignition/fuel/config.yaml << EOF
---
# The list of servers.
servers:
  -
    name: osrf
    url: https://fuel.ignitionrobotics.org

  # -
    # name: another_server
    # url: https://myserver

# Where are the assets stored in disk.
# cache:
#   path: /tmp/ignition/fuel
EOF

# Give user ownership of the environment folder
chown -R ubuntu /home/ubuntu/environment/

#TODO add rekognition label copy steps
