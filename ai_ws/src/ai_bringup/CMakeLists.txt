cmake_minimum_required(VERSION 2.8.3)
project(ai_bringup)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  tf
)

catkin_package(
   INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(ai_bringup 
               src/ai_bringup.cpp
               src/ai.cpp
               src/ai_linux_serial.cpp)
               
target_link_libraries(ai_bringup ${catkin_LIBRARIES})

