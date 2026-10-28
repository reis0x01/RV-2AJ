# RV-2AJ Robot Cell

ROS 2 control stack for a Mitsubishi MELFA RV-2AJ (CR1-571 controller, MELFA-BASIC IV) on Station 4
of a Festo MPS 500 transfer line. The controller is driven over RS-232 with direct-execution (EXEC)
commands, every move is validated by MoveIt before it reaches the arm, and the conveyor is
coordinated with an S7-1200 PLC over S7 (PUT/GET).

<img src="docs/images/real_robot_assembly.gif" width="100%" alt="RV-2AJ real assembly">

<img src="docs/images/gui_control_panel.png" width="100%" alt="RV-2AJ control panel">

## Architecture

```
 PyQt panel ─┐
 translator ─┼─► planner ──(MoveIt check)──► driver ──RS-232 8E2──► CR1-571 ──► RV-2AJ
 sequencer ──┘      ▲                          │
     ▲              └──── move_group ◄─────────┴─ /rv2aj/joint_states
     │
 plc_bridge ◄──── S7 / snap7, DB10 ────► S7-1200 ──► 3 stopper stations
```

| Package | Content |
|---|---|
| `rv2aj_description` | URDF/xacro, meshes, `config/joint_limits.yaml` (joint limits, frozen joints) |
| `rv2aj_moveit_config` | SRDF (group `arm`: `base_link` → `tool0`, state `home`), MoveIt config and launch |
| `rv2aj_interfaces` | `RobotStatus`, `SequencerStatus`, `TeachPoint`, services, `TcpMove` / `JointMove` actions |
| `rv2aj_control` | nodes, libraries, `config/rv2aj.yaml`, `launch/bringup.launch.py`, tests |

## Build and run

```bash
cd rv2aj_ws
source /opt/ros/lyrical/setup.bash        # or jazzy
colcon build && source install/setup.bash
cd src/rv2aj_control && python3 -m pytest -q test/
```

Dependencies: MoveIt 2, xacro, robot_state_publisher, `python3-pyqt5`, `python3-scipy`,
`python3-serial`, `python-snap7`.

```bash
./start_robot.sh                     # real controller on /dev/ttyUSB0
./start_robot.sh real /dev/ttyUSB1   # other port
./start_robot.sh sim                 # wire-level controller simulator
./stop_robot.sh                      # SIGINT → driver sends STOP, SRVOFF, CNTLOFF
```

The stack runs in a transient systemd scope (`rv2aj-stack`) and logs to `~/.ros/rv2aj_stack.log`.
Teach points are stored in `~/.ros/rv2aj_points.json` (sim: `~/.ros/rv2aj_sim/points.json`);
snapshots are in `points_backups/`.

Launch arguments: `use_fake_hardware`, `port`, `use_moveit` (default true), `gui`, `voice`,
`points_file`, `plc`.

## Robot

| Item | Value |
|---|---|
| Axes | J1 waist, J2 shoulder, J3 elbow, J5 wrist pitch, J6 wrist roll (no J4, reported as `****`) |
| Link lengths | base→J2 300 mm, J2→J3 250 mm, J3→J5 160 mm, J5→flange 72 mm, no lateral offsets |
| Base | mounted rotated 180° (180° yaw on the URDF base) |
| Gripper | Festo HGP-16 pneumatic, `EXECHOPEN 1` / `EXECHCLOSE 1`, collision box 80 × 80 × 120 mm |
| Serial | FTDI FT232, 9600 baud, 8 data bits, even parity, 2 stop bits |
| I/O | CN100: IN0–15, OUT0–15, hand I/O 900–907; OUT0–3 / IN0–5 are factory-dedicated |

With five axes the tool has pitch and roll but no yaw, so IK only reaches orientations in the
arm's vertical plane.

## Serial protocol

Frame: `1;1;<COMMAND>\r`, reply terminated by `\r`. Status prefix: `Qo` OK, `Qe`/`Qx` error;
lowercase (`Qok`, `Qer`) means an alarm is pending. Error replies embed the number
(`QeR422000000` → 4220); `ERRORMES<n>` returns the text.

| Command | Reply | Use |
|---|---|---|
| `JPOSF` | `QoKJ1;-149.72;J2;-64.62;J3;137.13;J4;****;J5;…` | joint angles, polled every cycle |
| `PPOSF` | `QoKX;84.78;Y;49.46;Z;522.81;A;…;B;…` | tool pose, controller frame |
| `STATE` | `QoK1.MB4;10;100;4;01060;…` | program, line, override %, last error |
| `VALM_INB(0)`, `VALM_OUTB(0)` | `QoKM_INB=+16` | I/O byte |
| `VALM_ERR`, `VALM_ERRNO` | `QoKM_ERRNO=+0` | active error |
| `CNTLON` / `CNTLOFF` | | operation rights, required before EXEC (else 6020) |
| `SRVON` / `SRVOFF`, `STOP`, `RSTALRM` | | servo, stop, alarm reset |
| `EXECJOVRD <1-100>` | | speed override |
| `EXECM_OUT(n)=0/1` | | write one output, read back to confirm |

Joint move, built from per-joint edits and verified before execution:

```
EXECJTMP=J_CURR
EXECJTMP.J1=JTMP.J1+RAD(5.00)      # one line per joint that changes
VALJTMP                            # compare with the target, abort on mismatch
EXECMOV JTMP
```

Controller behaviour the driver is built around:

- `EXECMOV` returns `QoK` immediately; completion is detected by polling `JPOSF`.
- `EXECMOV` during motion fails with 5640 (with or without `CNT 1`), so serial moves cannot be
  blended and every waypoint ends at zero velocity.
- Tuple expressions (`J_CURR+(…)`) give 4220. The simulator rejects them as well.
- The 5th `STATE` field is the last *recorded* error. A joint-range alarm appears as `81069x`
  (x = axis); whether an error is active comes from `VALM_ERRNO`.
- After `RSTALRM` the next move fails until the servo is cycled; `reset_alarm` sends
  `RSTALRM` and, if the servo was on, `SRVOFF` → `SRVON`.

## Nodes

Namespace `/rv2aj`.

| Node | Interfaces | Function |
|---|---|---|
| `driver` | pub `status`, `joint_states`; srv `connect`, `servo`, `stop`, `reset_alarm`, `set_override`, `gripper/open\|close`, `line/set`; act `tcp_move`, `joint_move` | owns the port; polls `JPOSF` every cycle, `STATE`/`PPOSF` every 1 s (0.5 s while moving), I/O bytes when idle |
| `planner` | act `planned/tcp_move`, `planned/joint_move`, `/rv2aj_arm_controller/follow_joint_trajectory` | path generation and MoveIt validation |
| `sequencer` | pub `sequencer/status`; srv `points/save\|delete\|goto\|clear\|set_speed\|set_io`, `loop/start\|stop\|auto`, `home/goto` | teach-and-repeat, home, auto-run |
| `plc_bridge` | pub `plc/part_ready`, `plc/state`; srv `plc/maintenance`, `plc/release_1..3`, `plc/line_stop`, `plc/line_reset` | DB10 exchange; idle while `plc_ip` is empty |
| `translator` | sub `voice/text` | text commands ("move up 20", "go to point 2") |
| `gui` | | operator panel |

`move_group` and `robot_state_publisher` run in the root namespace.

```bash
ros2 topic echo --once /rv2aj/status
ros2 action send_goal /rv2aj/planned/tcp_move rv2aj_interfaces/action/TcpMove "{axis: Z, distance_mm: 10.0}"
ros2 action send_goal /rv2aj/planned/joint_move rv2aj_interfaces/action/JointMove "{relative: true, joint: J1, offset_deg: 5.0}"
```

## Motion pipeline

1. **Path.** TCP moves: analytic 5-axis IK every 5 mm along the line with tool pitch and J6 held,
   reduced to the fewest waypoints within 1 mm of the line; a joint jump > 30° between samples is
   treated as a singularity and refused. Joint moves: direct joint-space line, or an OMPL plan
   shortcut to the fewest valid waypoints if the line collides.
2. **Validation.** Each waypoint and the interpolated path (≤ 1° steps) go through
   `/check_state_validity` (self-collision, table plane at Z = 0, gripper box) and the joint limits
   in `joint_limits.yaml`. Caps: 50 mm per TCP move, 45° per joint step.
3. **Execution.** Each waypoint is sent as an absolute `EXECMOV JTMP`, which is a straight line in
   joint space and therefore follows the validated path. Settle detection: *reached* = within
   tolerance and still for 0.2 s, *stopped* = still for 1 s away from target, timeout 120 s.

FK/IK are in `kinematics.py`. A joint listed in `frozen_joints` is held fixed by IK, the planner
and the driver.

## Sequencer

A teach point stores `joints[6]`, gripper state, `speed_percent` and an output list
(`io_outputs[]`, `io_values[]`). Playback per point *i*:

1. send `EXECJOVRD` with point *i*'s speed (only if it changed);
2. one absolute joint move, wait for settle;
3. command the gripper if its state changed (then wait `gripper_wait_s`), write the outputs.

Dwell per point is about 0.4 s, which is the deceleration to zero velocity forced by the
no-blending limit above.

Auto-run (`loop/auto`) starts one pass on the rising edge of *idle ∧ at home (all joints within 10°)
∧ trigger*. The trigger is `plc/part_ready` (`auto_trigger: plc`) or input IN1 (`auto_trigger:
input`). A pass runs from the point after `auto_start_point` (`P2E`, the home pose) to the last
point and returns home. STOP disarms it.

## Conveyor PLC link

The S7-1200 (program `plc/LineControl.scl`: DB10 `PC_Link` + FB `LineControl`, import via
*External source files → Generate blocks*) controls three stopper stations; cars circulate
1 → 2 → 3 → 1. Each station has an NC part sensor, an inductive car sensor and a stopper valve.

DB10 (non-optimized, PUT/GET enabled, 8 bytes). The laptop writes bytes 4–5 only:

| Address | Signal | Writer |
|---|---|---|
| DBX0.0–0.2 | PartReady, PLC_Heartbeat (1 Hz), MissedPart | PLC |
| DBW2 | PassCount | PLC |
| DBX4.0–4.4 | Busy, Done (2 s pulse), Error, Armed, Heartbeat | laptop |
| DBX5.0–5.5 | MaintMode, St1–St3_Release (pulse), LineStop, LineReset (pulse) | laptop |
| DBX6.0–6.5 | car / parts present, stations 1–3 | PLC |
| DBX7.0–7.4 | St1–St3 valve, MaintActive, LineStopped | PLC |

Handshake: `PartReady` → pass starts → `Busy` → PLC clears `PartReady` → arm returns home → `Done`
(or `Error`). Heartbeats run both ways with a 3 s timeout; no `PartReady` is forwarded while the
link is down, and `Done` is dropped on disconnect so a reconnect cannot count a pass twice.
`LineStop` is latched in the PLC and cleared only by a `LineReset` edge.

Station logic: St1 holds a car until the robot has emptied it (one retry, then release with
`MissedPart`); St2 holds hand-filled cars until St3 requests one; St3 holds full cars and releases
one per St1 release.

Configure `plc_ip` (port 102, rack 0, slot 1) in `rv2aj.yaml`. Offline test with the fake PLC:

```bash
python3 test/fake_plc.py [--stubborn]      # DB10 server on port 11020
ros2 run rv2aj_control plc_bridge --ros-args -r __ns:=/rv2aj -p plc_ip:=127.0.0.1 -p plc_port:=11020
```

## Tests

`python3 -m pytest -q test/` in `rv2aj_control` (built and sourced workspace required). Covers the
driver state machine and command syntax, FK/IK and straight-line planning, gripper, text parser and
the PLC handshake. `test/conftest.py` substitutes a generic robot so the real robot's
configuration does not affect results.
