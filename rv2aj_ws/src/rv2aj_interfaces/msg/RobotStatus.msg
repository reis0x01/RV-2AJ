# Published by the driver after every poll. Units are the controller's own:
# millimetres and degrees (joint_states carries SI units for other tools).
std_msgs/Header header
bool connected
bool servo_on
bool alarm
bool moving
string active_motion        # e.g. "TCP up 5 mm", empty when idle
int32 override_percent
float64[6] joints_deg       # J1..J6, NaN for a joint that is not fitted (J4)
float64 x_mm
float64 y_mm
float64 z_mm
float64 a_deg
float64 b_deg
string state_raw            # last raw STATE reply
string last_error           # why the link was dropped, empty if none
string gripper              # last commanded gripper state: unknown, open, closed
# Festo MPS 500 transfer line on connector XPA4 (bits 0-7 of the robot I/O).
# -1 = not read yet. IN6/IN7 = order bits; OUT4/OUT5 = error code, OUT6 = busy, OUT7 = station ready.
int32 line_inputs
int32 line_outputs
# Further I/O found in use 2026-10-16 (pneumatics on Station 4), -1 = not read yet:
int32 inputs_8_15           # general inputs IN8..IN15 (bit 0 = IN8)
int32 outputs_8_15          # general outputs OUT8..OUT15 (bit 0 = OUT8)
int32 hand_inputs           # hand inputs 900..907 (bit 0 = 900)
int32 hand_outputs          # hand outputs 900..907 (bit 0 = 900); 900/901 = gripper hand 1
