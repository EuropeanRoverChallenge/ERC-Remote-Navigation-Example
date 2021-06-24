cmake_minimum_required(VERSION 3.0.2)

project(erc_bringup)

find_package(catkin REQUIRED)

catkin_package()

install(
  DIRECTORY
    launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(
  PROGRAMS 
    scripts/image_saver
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)