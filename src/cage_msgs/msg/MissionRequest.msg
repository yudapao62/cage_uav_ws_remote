uint8 MISSION_FEEDER=0
uint8 MISSION_SCOUT=1

std_msgs/Header header
string mission_id
int32 target_cage_id
bool start
bool resume_after_override
bool rtl_after_mission
float64 feed_relative_alt
float64 transit_relative_alt
uint8 mission_type
bool auto_land
string task_map_uuid
uint64 task_map_version
string task_map_sha256
string task_manifest_sha256
