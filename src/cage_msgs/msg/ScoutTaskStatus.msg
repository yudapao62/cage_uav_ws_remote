std_msgs/Header header
string task_id
string mission_id
int32 cage_id
string state
string reason
string error_code

uint32 current_waypoint
uint32 total_waypoints
bool route_completed

string map_uuid
uint64 map_version
string geometry_sha256
uint32 confirmed_cells
uint32 expected_cells
float32 coverage
bool map_finalized
bool map_ready_for_feeding

bool rtl_commanded
bool rtl_accepted
bool returned_home
float32 home_horizontal_error_m
bool landed
bool disarmed

string output_directory
