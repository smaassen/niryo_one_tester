cmake_minimum_required(VERSION 2.8.3)
project(niryo_one_tester)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  niryo_one_msgs
  actionlib
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES robot_tester
  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


add_executable(simple_command_node src/simple_command.cpp)
target_link_libraries(simple_command_node
  ${catkin_LIBRARIES}
)

#add_executable(tower_stacking_demo_node src/tower_stacking_demo.cpp)
#target_link_libraries(tower_stacking_demo_node
#  ${catkin_LIBRARIES}
#)

