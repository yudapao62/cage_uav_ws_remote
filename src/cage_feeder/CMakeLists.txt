cmake_minimum_required(VERSION 3.0.2)
project(cage_feeder)
find_package(catkin REQUIRED COMPONENTS cage_msgs mavros_msgs rospy std_msgs)
catkin_python_setup()
catkin_package()
catkin_install_python(PROGRAMS
  scripts/feeder_rc9_node.py
  scripts/feeder_simulator_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_rc9_logic.py)
endif()
