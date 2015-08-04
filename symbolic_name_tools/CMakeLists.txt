cmake_minimum_required(VERSION 2.8.3)
project(symbolic_name_tools)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package(
  INCLUDE_DIRS include LIBRARIES symbolic_name_tools #  CATKIN_DEPENDS roscpp
 #  DEPENDS system_lib

)

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(symbolic_name_tools
  src/symbolic_name_tools.cpp
)

