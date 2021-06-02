cmake_minimum_required(VERSION 3.0.2)
project(px100_simulation)

find_package(catkin REQUIRED)
catkin_package()

install(PROGRAMS
  nodes/px100.py
  nodes/init.py
  nodes/collect.py
  nodes/aws_utilities.py
  nodes/generate_worlds.py
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
