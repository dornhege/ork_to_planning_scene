cmake_minimum_required(VERSION 2.8.3)
project(ork_to_planning_scene)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  ecto
  ecto_image_pipeline
  ecto_ros
  moveit_msgs
  moveit_ros_move_group
  object_recognition_core
  object_recognition_msgs
  object_recognition_ros
  ork_to_planning_scene_msgs
  roscpp
  rospy
  symbolic_name_tools
  tf
  visualization_msgs
)

find_package(OpenCV REQUIRED)

catkin_python_setup()

#   moveit_msgs#   object_recognition_msgs#   ork_to_planning_scene_msgs

catkin_package(
#  INCLUDE_DIRS include
 #  LIBRARIES ork_to_planning_scene
 #  CATKIN_DEPENDS actionlib moveit_msgs moveit_ros_move_group object_recognition_msgs ork_to_planning_scene_msgs roscpp symbolic_name_tools tf
 #  DEPENDS system_lib
 CATKIN_DEPENDS actionlib moveit_msgs object_recognition_core object_recognition_msgs ork_to_planning_scene_msgs rospy symbolic_name_tools tf visualization_msgs
)

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(ork_to_planning_scene src/ork_to_planning_scene.cpp src/main.cpp)

add_dependencies(ork_to_planning_scene ork_to_planning_scene_msgs_generate_messages_cpp)

target_link_libraries(ork_to_planning_scene
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

