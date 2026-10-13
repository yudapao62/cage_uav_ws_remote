cmake_minimum_required(VERSION 3.0.2)
project(cage_control_authority)

find_package(catkin REQUIRED COMPONENTS
  mavros_msgs
  rospy
  sensor_msgs
  std_msgs
  std_srvs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/control_authority_manager.py
  scripts/joystick_rc_bridge.py
  scripts/keyboard_joy_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
