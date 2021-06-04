cmake_minimum_required(VERSION 3.0.2)
project(robomaker_workshop)

find_package(catkin REQUIRED)
catkin_package()

install(PROGRAMS
  scripts/px100.py
  scripts/init.py
  scripts/collect.py
  scripts/aws_utilities.py
  scripts/generate_worlds.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  config
  launch
  media
  meshes
  models
  rviz
  urdf
  worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
