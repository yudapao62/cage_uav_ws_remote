cmake_minimum_required(VERSION 3.0.2)
project(cage_bringup)
find_package(catkin REQUIRED)
catkin_package()
catkin_install_python(PROGRAMS
  scripts/evaluate_persistent_lattice.py
  scripts/persistent_lattice_acceptance.py
  scripts/persistent_lattice_event_logger.py
  scripts/publish_demo_mission.py
  scripts/simulation_acceptance.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
