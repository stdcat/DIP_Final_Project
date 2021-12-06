cmake_minimum_required(VERSION 3.0.2)
project(dip_final)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

find_package(OpenCV REQUIRED)

find_package(Eigen3 REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES dzy
#  CATKIN_DEPENDS OpenCV roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

set( ThirdParty
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES}
        ${Eigen_LIBRARIES}
)

add_library(
        dipLib
        include/dipLib/dipLib.cpp
)

add_executable(dip_final src/dip.cpp)
target_link_libraries(dip_final
    ${ThirdParty}
    dipLib
)