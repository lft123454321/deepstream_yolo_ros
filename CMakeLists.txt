cmake_minimum_required(VERSION 3.0.2)
project(deepstream_yolo_ros)





find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)

add_message_files(
  FILES
  BoundingBox.msg
  BoundingBoxes.msg
  MultiCamResult.msg
)

generate_messages(
  DEPENDENCIES std_msgs
)

catkin_package(
  INCLUDE_DIRS
  LIBRARIES 
  CATKIN_DEPENDS roscpp std_msgs message_runtime
)
