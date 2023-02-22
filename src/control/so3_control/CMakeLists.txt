cmake_minimum_required(VERSION 3.0.2)
project(so3_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  uav_state
)

find_package(Eigen3 REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(
  ${catkin_INCLUDE_DIRS}
)

include_directories(include)##让自己的包的c++文件能找到头文件

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES so3_control
  CATKIN_DEPENDS roscpp rospy std_msgs uav_state
#  DEPENDS system_lib
)

add_library(${PROJECT_NAME} src/SO3Control.cpp)
target_link_libraries(${PROJECT_NAME} 
${catkin_LIBRARIES}
)


