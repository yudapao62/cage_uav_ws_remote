uint8 EDGE_INFERRED=0
uint8 EDGE_OBSERVED=1

int32 local_i
int32 local_j
geometry_msgs/Point[] quad_uv
geometry_msgs/Point projected_center
float64[4] projected_covariance_xy
uint8[4] edge_state
float32 confidence
bool expected_visible
