cmake_minimum_required(VERSION 3.0.2)
project(line_follower)

set(CMAKE_CXX_FLAGS "-fpermissive -std=c++0x")

find_package(catkin REQUIRED COMPONENTS
  rospy
  dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
  cfg/LineParam.cfg
)
catkin_package(DEPENDS)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

