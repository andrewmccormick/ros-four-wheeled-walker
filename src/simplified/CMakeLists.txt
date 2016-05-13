cmake_minimum_required(VERSION 2.8.3)
project(simplified)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  controller_manager
  urdf
  genmsg
  geometry_msgs
  roscpp
  rospy
  std_msgs
  tf
  move_base
  gazebo_ros
)
find_package(gazebo REQUIRED)
###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES walkernew
#  CATKIN_DEPENDS controller_manager urdf
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} ./gazebo_plugins/include)

## Declare a Gazebo plugin (C++ library)
add_library(wheel_encoder_plugin gazebo_plugins/src/wheel_encoder.cpp)
target_link_libraries(wheel_encoder_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

## Declare a C++ executable
# add_executable(walkernew_node src/walkernew_node.cpp)
add_executable(base_controller src/base_controller.cpp)


## Specify libraries to link a library or executable target against
target_link_libraries(base_controller ${catkin_LIBRARIES})


