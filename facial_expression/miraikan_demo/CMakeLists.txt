cmake_minimum_required(VERSION 2.8.3)
project(miraikan_demo)

find_package(catkin REQUIRED COMPONENTS std_msgs)

catkin_package()

install(DIRECTORY euslisp launch scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
  )

include_directories()
