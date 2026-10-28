string name
float64[6] joints_deg       # J4 is 0 on this 5-axis robot
string gripper              # "open" / "closed": actuated after reaching the point; "" = leave as is
int32 speed_percent         # this point's own override 1-100%; a move uses the slower of its two points
# Station outputs set (in order) after reaching the point, same slot as gripper above
# (0-15 general, 900-907 hand/pneumatic; see driver's line/set). Parallel arrays.
int32[] io_outputs
bool[] io_values
