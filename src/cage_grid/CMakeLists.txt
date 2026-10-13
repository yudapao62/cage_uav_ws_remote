cmake_minimum_required(VERSION 3.0.2)
project(cage_grid)
find_package(catkin REQUIRED COMPONENTS cage_geo cage_msgs geometry_msgs rospy sensor_msgs std_msgs visualization_msgs)
catkin_python_setup()
catkin_package(CATKIN_DEPENDS cage_geo cage_msgs geometry_msgs rospy sensor_msgs std_msgs visualization_msgs)
catkin_install_python(PROGRAMS
  scripts/cage_grid_node.py
  scripts/feeder_identity_node.py
  scripts/scout_map_registry_node.py
  scripts/scout_lattice_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test)
endif()
