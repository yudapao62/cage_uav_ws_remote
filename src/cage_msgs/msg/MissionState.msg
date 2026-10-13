uint8 MISSION_FEEDER=0
uint8 MISSION_SCOUT=1

std_msgs/Header header
string mission_id
string state
string reason
int32 current_waypoint
bool feeder_enabled
bool manual_override
string current_cell_id
uint8 mission_type
bool auto_land
