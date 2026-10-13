cmake_minimum_required(VERSION 3.0.2)
project(cage_sitl)

find_package(catkin REQUIRED COMPONENTS
  cage_control_authority
  gazebo_ros
  mavros
  rospy
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/analyze_takeover_bag.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS
  scripts/start_gazebo_classic.sh
  scripts/start_ardupilot_sitl.sh
  scripts/check_sitl.sh
  scripts/check_manual_sitl.sh
  scripts/record_manual_takeover_test.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
