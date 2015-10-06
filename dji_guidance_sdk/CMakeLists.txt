cmake_minimum_required(VERSION 2.8.3)
project(dji_guidance_sdk)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  image_transport
  #opencv2
  roscpp
  rospy
  sensor_msgs
  std_msgs
  message_generation
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenCV REQUIRED)
message(STATUS "Found OpenCV: ${OpenCV_LIBS}" )


## code to find GUIDANCE library
FIND_PATH(GUIDANCE_INCLUDE_DIR DJI_guidance.h onboard.h
	${PROJECT_SOURCE_DIR}/src/guidance_lib
 )
message(STATUS "Found Guidance IncludeFiles: ${GUIDANCE_INCLUDE_DIR}")

FIND_LIBRARY(GUIDANCE_LIBRARY NAMES GUIDANCE libDJI_guidance.so
  /usr/local/lib
)
message(STATUS "Found Guidance library: ${GUIDANCE_LIBRARY}")
#########################################################
FIND_PATH(LIBUSB_1_INCLUDE_DIR libusb.h
   /usr/include/libusb-1.0
)
message(STATUS "Found libusb.h: ${LIBUSB_1_INCLUDE_DIR}")
FIND_LIBRARY(LIBUSB_1_LIBRARY NAMES usb-1.0 
   /usr/lib/x86_64-linux-gnu
)
message(STATUS "Found libusb 1.0 library: ${LIBUSB_1_LIBRARY}")
if (LIBUSB_1_INCLUDE_DIR AND LIBUSB_1_LIBRARY)
   set(LIBUSB_1_FOUND TRUE)
   set(LIBUSB_1_LIBRARIES ${LIBUSB_1_LIBRARY})
else (LIBUSB_1_INCLUDE_DIR AND LIBUSB_1_LIBRARY)
   set(LIBUSB_1_FOUND FALSE)
endif (LIBUSB_1_INCLUDE_DIR AND LIBUSB_1_LIBRARY)
if (LIBUSB_1_FOUND)
      message(STATUS "Found LibLIBUSB_1: ${LIBUSB_1_LIBRARIES}")
else (LIBUSB_1_FOUND)
	   message(STATUS "don't find LibLIBUSB_1")
endif (LIBUSB_1_FOUND)
MARK_AS_ADVANCED(LIBUSB_1_INCLUDE_DIR LIBUSB_1_LIBRARIES LIBUSB_1_LIBRARY)
#########################################################
## find guidance lib

include_directories("${PROJECT_SOURCE_DIR}/src/guidance_lib")


###############################################################

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
#add_message_files(
#  FILES
#  trackInfo.msg
#  )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
#   geometry_msgs
# )

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
#  LIBRARIES cv_uav
  CATKIN_DEPENDS roscpp rospy
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
${catkin_INCLUDE_DIRS}
${OpenCV_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(cv_uav
#   src/${PROJECT_NAME}/cv_uav.cpp
# )

## Declare a cpp executable


add_executable(dji_guidance_node  src/dji_guidance_node.cpp )
target_link_libraries(dji_guidance_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${GUIDANCE_LIBRARY} ${LIBUSB_1_LIBRARIES}  )


add_executable(dji_guidance_test src/dji_guidance_test.cpp )
target_link_libraries(dji_guidance_test ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${GUIDANCE_LIBRARY} ${LIBUSB_1_LIBRARIES}  )




