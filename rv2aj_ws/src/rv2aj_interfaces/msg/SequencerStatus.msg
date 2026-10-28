TeachPoint[] points
bool loop_active
int32 cycle
string current_point        # point being moved to, empty when idle
bool auto_mode              # auto-run armed (runs a pass per trigger while parked at home)
int32 pending_parts         # unused with the TCP sensor (no queue); kept for compatibility
int32 auto_input_bit        # which input the auto-run watches (default IN1, TCP sensor)
string auto_trigger         # what starts an auto pass: "input" (TCP sensor bit) or "plc" (conveyor PLC)
int32 auto_pass_count       # auto passes finished (ok or not) since the sequencer started
bool last_pass_ok           # result of the last finished auto pass
