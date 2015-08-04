cmake_minimum_required(VERSION 2.8.3)
project(ork_to_planning_scene_msgs)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  message_generation
  moveit_msgs
  object_recognition_msgs
)

add_action_files(
  FILES
  UpdatePlanningSceneFromOrk.action
)

generate_messages(
  DEPENDENCIES
  moveit_msgs
  object_recognition_msgs
)

catkin_package(
#  INCLUDE_DIRS include
 #  LIBRARIES ork_to_planning_scene_msgs
 CATKIN_DEPENDS #  DEPENDS system_lib
 actionlib message_runtime moveit_msgs object_recognition_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

