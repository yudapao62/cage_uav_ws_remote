cmake_minimum_required(VERSION 3.0.2)
project(cage_simulation)
find_package(catkin REQUIRED COMPONENTS cage_geo cage_grid cage_msgs cage_waypoint_planner gazebo_msgs geometry_msgs mavros_msgs nav_msgs rospy sensor_msgs std_msgs)
catkin_package(CATKIN_DEPENDS cage_geo cage_grid cage_msgs cage_waypoint_planner gazebo_msgs geometry_msgs mavros_msgs nav_msgs rospy sensor_msgs std_msgs)
catkin_install_python(PROGRAMS
  scripts/cage_ground_truth_publisher.py
  scripts/mock_mavros_node.py
  scripts/scout_task_manager_node.py
  scripts/scout_route_commander.py
  scripts/synthetic_lattice_observation_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY launch worlds models config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(PROGRAMS
  scripts/start_full_simulation.sh
  scripts/generate_circular_cage.py
  scripts/generate_fish_farm.py
  scripts/generate_multi_cage_world.py
  scripts/generate_water_surface.py
  scripts/generate_uav_camera.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
