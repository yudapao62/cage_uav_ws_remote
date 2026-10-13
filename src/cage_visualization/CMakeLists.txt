cmake_minimum_required(VERSION 3.0.2)
project(cage_visualization)

find_package(catkin REQUIRED COMPONENTS
  cage_geo
  cage_msgs
  geometry_msgs
  mavros_msgs
  nav_msgs
  rospy
  sensor_msgs
  std_msgs
  visualization_msgs
)

catkin_python_setup()

catkin_package(CATKIN_DEPENDS
  cage_geo
  cage_msgs
  geometry_msgs
  mavros_msgs
  nav_msgs
  rospy
  sensor_msgs
  std_msgs
  visualization_msgs
)

catkin_install_python(PROGRAMS
  scripts/cage_marker_node.py
  scripts/waypoint_marker_node.py
  scripts/trajectory_visualizer.py
  scripts/mission_status_visualizer.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test)
endif()
