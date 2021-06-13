sudo apt update
sudo DEBIAN_FRONTEND=noninteractive apt upgrade -yq

aws configure set profile.robomaker_workshop.role_arn arn:aws:iam::517502204741:role/ResourcesForRoboticsWorkshop
aws configure set profile.robomaker_workshop.source_profile default
aws configure set profile.robomaker_workshop.region eu-central-1

mkdir -p ~/environment/aws_ws/src/
cd ~/environment/aws_ws/src/
git clone https://github.com/adi3/robomaker_workshop

cd ~/environment/aws_ws
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro melodic -r -y

cd ~/environment
curl 'https://raw.githubusercontent.com/Interbotix/interbotix_ros_manipulators/main/interbotix_ros_xsarms/install/amd64/xsarm_amd64_install.sh' > xsarm_amd64_install.sh
chmod +x xsarm_amd64_install.sh
echo "N" | ./xsarm_amd64_install.sh
rm xsarm_amd64_install.sh

source ~/.bashrc
echo "~/environment/aws_ws/src/robomaker_workshop/devel/setup.bash" >> ~/.bashrc

sudo apt install python-catkin-tools imagemagick -y
pip install boto3

cd ~/environment/aws_ws
catkin build
source ~/environment/aws_ws/devel/setup.bash

export DISPLAY=:0

aws s3 cp s3://adsnghw-robotics/px100-dataset/real_coins.zip /tmp/px100-dataset.zip --profile robomaker_workshop
unzip /tmp/px100-dataset.zip -d /tmp/px100-dataset
BUCKET=px100-dataset-$(cut -d'-' -f1 <<< $(uuidgen))
aws s3 cp /tmp/px100-dataset s3://$BUCKET/assets/px100-dataset --recursive
