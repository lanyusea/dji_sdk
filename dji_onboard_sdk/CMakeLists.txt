cmake_minimum_required(VERSION 2.8.3)
project(dji_onboard_sdk)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
 std_msgs
 geometry_msgs
 message_generation
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################


## Generate messages in the 'msg' folder
add_message_files(
  FILES

  api_angular_v.msg
  api_gps_data.msg
  api_linear_a.msg
  api_linear_v.msg
  api_orientation.msg
  api_gimbal.msg
  uav_sensor_data.msg

  api_cmd_data.msg
  uav_ctrl_data.msg
  gimbal_ctrl_data.msg
  )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  #std_msgs
  geometry_msgs
  )


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
  INCLUDE_DIRS
  LIBRARIES serial_to_uav
  CATKIN_DEPENDS roscpp std_msgs rospy
  DEPENDS system_lib
)
 
###########
## Build ##  
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)

file(GLOB DJI_SDK_LIB_SOURCES src/dji_sdk_lib/*.cpp) 
file(GLOB DJI_SDK_INFO_LIB_SOURCES src/dji_info_lib/*.cpp) 

##SET_SOURCE_FILES_PROPERTIES(${DJI_SDK_LIB_SOURCES} PROPERTIES LANGUAGE CXX)




include_directories(
  ${catkin_INCLUDE_DIRS}
  "${PROJECT_SOURCE_DIR}/src/sdk_lib"
)

## Declare a cpp library
# add_library(serial_to_uav
#   src/${PROJECT_NAME}/serial_to_uav.cpp
# ) 

## Declare a cpp executable
add_executable(serial_to_N1 src/serial_to_N1.cpp ${DJI_SDK_LIB_SOURCES} ${DJI_SDK_INFO_LIB_SOURCES})
target_link_libraries(serial_to_N1   ${catkin_LIBRARIES} )



