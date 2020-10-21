cmake_minimum_required(VERSION 3.0.2)
project(cob_control_msgs)
find_package(catkin REQUIRED COMPONENTS geometry_msgs message_generation std_msgs)

add_message_files(
  FILES
    ObstacleDistance.msg
    ObstacleDistances.msg
)

add_service_files(
  FILES
    GetObstacleDistance.srv
)

generate_messages(
  DEPENDENCIES
    geometry_msgs
    std_msgs
)

catkin_package(CATKIN_DEPENDS geometry_msgs message_runtime std_msgs)
