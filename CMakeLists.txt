cmake_minimum_required(VERSION 2.8.3)
project(wc_perception)

find_package(catkin REQUIRED COMPONENTS
# find_package(catkin ALL_DEPS_REQUIRED # JKM - tried this
  roscpp
  rospy
  std_msgs
  message_generation
  wc_msgs REQUIRED #JKM
  catkin_simple REQUIRED
)

catkin_python_setup()
catkin_simple()


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

cs_export()
