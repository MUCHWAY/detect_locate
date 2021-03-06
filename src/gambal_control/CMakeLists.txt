cmake_minimum_required(VERSION 3.0.2)
project(gambal_control)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
# set(OpenCV_DIR /usr/lib/aarch64-linux-gnu/cmake/opencv4/)
set(OpenCV_DIR /usr/local/lib/cmake/opencv4/)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  yolov5_detect
  OpenCV
  message_generation
  geometry_msgs
)

add_message_files(
  FILES
  camera_attitude.msg
)

#generate_messages必须在catkin_package前面
generate_messages(
 DEPENDENCIES
 geometry_msgs
)

catkin_package(
CATKIN_DEPENDS
 INCLUDE_DIRS include
#  LIBRARIES gambal_control
#  CATKIN_DEPENDS roscpp rospy std_msg
#  DEPENDS system_lib
)



## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# find_package(OpenCV)
# include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(gambal_control  src/control.cpp src/ClientSocket.cpp src/Socket.cpp)
add_dependencies(gambal_control ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(gambal_control ${OpenCV_LIBS} ${catkin_LIBRARIES})

add_executable(gambal_control_gazebo  src/control_gazebo.cpp)
add_dependencies(gambal_control_gazebo ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(gambal_control_gazebo ${OpenCV_LIBS} ${catkin_LIBRARIES})


