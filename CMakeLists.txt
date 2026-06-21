cmake_minimum_required(VERSION 3.0.2)
project(robot_bt_action)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  geometry_msgs
  navigation
  nav_msgs
  rospy
  sensor_msgs
  std_msgs
  message_generation
)

add_service_files(
  FILES
  GetTarget.srv
  NavigateToPose.srv
  SetOffset.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS actionlib actionlib_msgs message_runtime geometry_msgs navigation nav_msgs sensor_msgs std_msgs rospy
)

# 如果你的 scripts 目录里有 Python 节点，也可以保留这些（非必须）
catkin_install_python(PROGRAMS
  scripts/control.py
  scripts/run_bt.py
  scripts/bt_navigation_server.py
  scripts/bt_target_server.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
