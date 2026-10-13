cmake_minimum_required(VERSION 3.0.2)
project(cage_msgs)
find_package(catkin REQUIRED COMPONENTS geometry_msgs message_generation std_msgs)
add_message_files(FILES
  CageGeometry.msg
  CageGrid.msg
  CellTargetArray.msg
  CellTarget.msg
  FeederCellHypothesis.msg
  FeederCellHypothesisArray.msg
  FeedingTaskRequest.msg
  FeedingTaskStatus.msg
  FeederIdentityState.msg
  LatticeCell.msg
  LatticeEdge.msg
  LocalLatticeCell.msg
  LocalLatticeObservation.msg
  MissionRequest.msg
  MissionWaypoint.msg
  MissionWaypointArray.msg
  MissionState.msg
  ScoutTaskRequest.msg
  ScoutTaskStatus.msg
  ScoutMapSummary.msg
  ScoutMapActivationStatus.msg
  FeederCommand.msg
  OffsetHypothesis.msg
  OffsetHypothesisArray.msg
  PersistentLattice.msg)
add_service_files(FILES
  ActivateScoutMap.srv
  FinalizeScoutMap.srv
  ListScoutMaps.srv
  ManageScoutMapLease.srv
  ResetScoutMap.srv
  RunFeeder.srv)
generate_messages(DEPENDENCIES geometry_msgs std_msgs)
catkin_package(CATKIN_DEPENDS geometry_msgs message_runtime std_msgs)
