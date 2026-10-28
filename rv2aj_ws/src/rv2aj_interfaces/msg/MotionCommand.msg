# What the translator understood from one spoken phrase.
string KIND_STOP=stop
string KIND_TCP=tcp
string KIND_GOTO_POINT=goto_point
string KIND_HOME=home
string KIND_START_LOOP=start_loop
string KIND_END_LOOP=end_loop
string KIND_NONE=none

string kind
string axis                 # tcp: X, Y, Z or R (along the arm's reach)
float64 distance_mm         # tcp: signed distance
string point                # goto_point: point name, e.g. P2
string text                 # the phrase as recognised
string reason               # kind == none: why it was ignored
