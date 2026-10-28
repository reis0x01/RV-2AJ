# RV-2AJ Robot Cell

Control software for a Mitsubishi MELFA RV-2AJ arm (controller CR1-571) on Station 4 of a
Festo MPS 500 transfer line. It has three parts:

- a ROS 2 + MoveIt stack that drives the controller over RS-232,
- a PyQt control panel for the operator,
- a link to the S7-1200 PLC that runs the conveyor stoppers.

The real robot running the taught assembly sequence (sped up about 19×):

<img src="docs/images/real_robot_assembly.gif" width="100%" alt="RV-2AJ real assembly">

The control panel:

<img src="docs/images/gui_control_panel.png" width="100%" alt="RV-2AJ control panel">

## Contents

1. [Quick start](#1-quick-start)
2. [Hardware](#2-hardware)
3. [Repository layout](#3-repository-layout)
4. [Serial protocol](#4-serial-protocol)
5. [Robot calibration](#5-robot-calibration)
6. [ROS 2 stack](#6-ros-2-stack)
7. [How a move is executed](#7-how-a-move-is-executed)
8. [Control panel](#8-control-panel)
9. [Transfer line and PLC link](#9-transfer-line-and-plc-link)

---

## 1. Quick start

### Install and build

```bash
git clone git@github.com:reis0x01/RV-2AJ.git
cd RV-2AJ/rv2aj_ws
source /opt/ros/lyrical/setup.bash        # or jazzy
colcon build && source install/setup.bash
cd src/rv2aj_control && python3 -m pytest -q test/
```

Requirements: ROS 2 (Lyrical or Jazzy) with MoveIt 2, xacro and robot_state_publisher, plus
`python3-pyqt5`, `python3-scipy`, `python3-serial`, and `python-snap7` for the PLC link.

Teach points are not part of the repository. The stack reads them from `~/.ros/rv2aj_points.json`;
copy one of the files in `points_backups/` there to start with the taught sequence.

### Start and stop

```bash
./start_robot.sh                     # real robot on /dev/ttyUSB0
./start_robot.sh sim                 # simulated controller, no hardware
./start_robot.sh real /dev/ttyUSB1   # real robot on another port
./stop_robot.sh
```

The panel opens after 15–20 s. The stack runs in its own systemd scope (`rv2aj-stack`), so closing
the terminal does not stop it. The start script refuses to run if the serial port is missing or the
stack is already running, and picks whichever ROS 2 is installed under `/opt/ros`.

Always stop with `./stop_robot.sh`. It sends SIGINT first so the driver can send STOP, SRVOFF and
CNTLOFF to the controller, then kills whatever is left and waits until the scope is gone.

### Using it

1. Check that the header shows CONNECTED and NO ALARM.
2. Press Servo ON. Only a person does this; no code ever switches the servo on.
3. Move the arm with the jog buttons, Home, taught points or the TCP arrows.
4. To stop: `+`, Esc, the STOP button, or the hardware E-stop.

After an alarm, press Reset alarm. The driver then cycles the servo off and on, because on this
robot the first move after an alarm reset fails otherwise.

The full stack output goes to `~/.ros/rv2aj_stack.log`.

---

## 2. Hardware

### Arm: MELFA RV-2AJ

- 5 axes: J1 waist, J2 shoulder, J3 elbow, J5 wrist pitch, J6 wrist roll. There is no J4 (the
  controller reports it as `****`), so the tool can pitch and roll but not yaw.
- Geometry (manual BFP-A8050, checked against the controller's positions to within 0.05 mm):
  shoulder axis 300 mm above the mounting face, upper arm 250 mm, forearm 160 mm,
  J5 axis to flange 72 mm, no lateral offsets.
- Absolute encoders with battery backup. When the batteries run low the origin is lost and the
  controller refuses every EXEC command with error 5150.
- The base is mounted turned 180° from the manual's drawing; the URDF base has a 180° yaw for this.

### Controller: CR1-571

- MELFA-BASIC IV. Commands are run directly over RS-232 as EXEC commands.
- The controller's own joint limits (parameter MEJAR, set on the teach pendant) are the final
  authority. They cannot be read over serial.
- Standard I/O is on the 50-pin CN100 connector. OUT0–OUT3 and IN0–IN5 are factory-dedicated
  (running, servo on, error, operation rights / stop, servo off, reset, start, servo on, operation rights).

### Gripper: Festo HGP-16

Pneumatic parallel gripper, 160 N at 6 bar, driven with `EXECHOPEN 1` / `EXECHCLOSE 1`.
The air hoses run around the wrist, so watch them when J6 turns a lot.

### Cell

The robot sits on the Station 4 table ("Robot with assembly"), connected to the station through
the SysLink connector XPA4. A blue block, a chute, pneumatic parts and the conveyor rails stick up
above the table. Only the flat table is in the collision model, so moves near those parts need watching.

### PC link

FTDI FT232 USB-serial adapter on `/dev/ttyUSB0`, 9600 baud, 8 data bits, even parity, 2 stop bits.
The user must be in the `dialout` group. The adapter occasionally drops off the bus.

---

## 3. Repository layout

```
RV-2AJ/
├── start_robot.sh, stop_robot.sh
├── rv2aj_ws/src/
│   ├── rv2aj_description/     URDF/xacro, meshes, config/joint_limits.yaml
│   ├── rv2aj_moveit_config/   SRDF, MoveIt config and launch
│   ├── rv2aj_interfaces/      messages, services, actions
│   └── rv2aj_control/         nodes, libraries, config/rv2aj.yaml, launch, tests
├── plc/LineControl.scl        S7-1200 program for the stopper stations
├── docu/                      write-ups (S7-1200 retrofit, robot control methods)
├── docs/images/               images used in this README
└── points_backups/            teach-point snapshots
```

Runtime files are written to `~/.ros/`:

| File | Content |
|---|---|
| `rv2aj_points.json` | taught points (real robot) |
| `rv2aj_sim/points.json` | the simulator's own points |
| `rv2aj_stack.log` | stack output |
| `rv2aj_joint_limit_probe.txt` | joint-limit probe results |

---

## 4. Serial protocol

### Framing

- Every command is sent as `1;1;<COMMAND>\r`; replies end with `\r`.
- `Qo…` means OK, `Qe…` / `Qx…` means error. A lowercase status (`Qok`, `Qer`) means an alarm is pending.
- Error replies carry the number: `QeR422000000` is error 4220. `ERRORMES<n>` returns its text.

### Queries

| Command | Example reply | Meaning |
|---|---|---|
| `STATE` | `QoK1.MB4;10;100;4;01060;1;1;;;…` | program, line, override %, …, last error |
| `JPOSF` | `QoKJ1;-149.72;J2;-64.62;J3;137.13;J4;****;…` | joint angles |
| `PPOSF` | `QoKX;84.78;Y;49.46;Z;522.81;A;-221.18;B;-20.22;…` | tool pose in the controller frame |
| `VALM_INB(0)` / `VALM_OUTB(0)` | `QoKM_INB=+16` | I/O byte (bits 0–7) |
| `VALM_IN(n)` / `VALM_OUT(n)` | `QoKM_OUT(4)=+0` | single bit |
| `VALM_ERR`, `VALM_ERRNO` | `QoKM_ERRNO=+0` | active error |
| `VALJTMP`, `VALPTMP` | `QoKJTMP=(-156.60,-6.49,…)` | read back a variable |

The fifth field of `STATE` is the last recorded error, not necessarily an active one. Use
`VALM_ERR` / `VALM_ERRNO` to check for an active error. A joint-range alarm shows up there as
`81069x`, where x is the axis (`8106961` = J1, `…62` = J2, and so on).

### Commands

| Purpose | Commands |
|---|---|
| Operation rights | `CNTLON` / `CNTLOFF` (required before any EXEC command) |
| Servo | `SRVON` / `SRVOFF` |
| Stop | `STOP` (also halts a running `EXECMOV`) |
| Reset alarm | `RSTALRM` |
| Speed override | `EXECJOVRD <1-100>` |
| Joint move | `EXECJTMP=J_CURR`, `EXECJTMP.J<n>=JTMP.J<n>+RAD(<deg>)` per joint, `VALJTMP`, `EXECMOV JTMP` |
| Straight TCP move | `EXECPTMP=P_CURR`, `EXECPTMP.Z=PTMP.Z+5.00`, `VALPTMP`, `EXECMOV PTMP` |
| Gripper | `EXECHOPEN 1` / `EXECHCLOSE 1` |
| Output | `EXECM_OUT(n)=0/1` |

Things to know:

- `EXECMOV` answers immediately and the arm keeps moving, so the driver polls `JPOSF` until it settles.
- A new `EXECMOV` while one is running is refused with error 5640, with or without `CNT 1`. Moves
  over serial cannot be blended; the arm stops at every point.
- Tuple expressions such as `J_CURR+(…)` give a syntax error (4220). Absolute moves are therefore
  built from per-joint edits, and the simulator rejects tuple forms too.
- Every move is read back with `VAL…` and compared before `EXECMOV`; a mismatch cancels it.
- After any alarm reset the next move fails unless the servo is switched off and on again. The
  driver's `reset_alarm` does this.

### Error numbers

| Error | Text | Usual cause |
|---|---|---|
| 0965 | Excessive error 1 | servo could not follow |
| 1120 | Encoder ABS position data lost | encoder batteries |
| 4220 | Syntax error | unsupported expression |
| 5150 | The origin has not been set | origin lost, all EXEC commands refused |
| 5600 | Cannot execute during an error | reset the alarm first |
| 5640 | Cannot execute during RUN | a move is still running |
| 6020 | The Operation is disable | `CNTLON` missing |
| `81069x` | joint range limit, axis x | MEJAR limit reached |

---

## 5. Robot calibration

Everything specific to this arm is in one file,
`rv2aj_ws/src/rv2aj_description/config/joint_limits.yaml`: joint limits, zero offsets and frozen
joints. The URDF, the kinematics library and the panel all read it. After changing it, rebuild
(`colcon build --packages-select rv2aj_description rv2aj_control`) and restart.

### Joint limits

The software limits are currently set to ±720° on every joint, which turns them off; the
controller's MEJAR limits are the ones that stop the arm. The measured values below are kept as
comments next to each joint in `joint_limits.yaml` and can be put back by copying them in.

They were measured with `test/probe_joint_limits.py`, which steps one joint 5° at a time at 5 %
speed until the controller alarms, and set about 2° inside the alarm points.

| Joint | Alarm points | Measured limit | Note |
|---|---|---|---|
| J1 | −329.7 / −19.5 | −327.5 … −21.5 | |
| J2 | −152 / +2.7 | −150 … +0.5 | negative side stopped by the table |
| J3 | −44.7 / +181.6 | −43 … +210 | upper limit opened after widening MEJAR |
| J5 | −274 / −82.0 | −272 … −84 | |
| J6 | none in −425 … +339 | −425 … +335 | |

With limits active, a joint that is already outside its range may stay there or move back in,
but never further out.

The probe (`python3 test/probe_joint_limits.py J3 +`) bypasses the planner on purpose and has its own
table guard. Someone has to stand at the E-stop while it runs.

### Home

Home is the pick/wait pose next to the parts, which is also the taught point P2E. It is set in
`rv2aj.yaml` (sequencer `home_joints_deg`) and in the SRDF group state `home`.

| J1 | J2 | J3 | J5 | J6 |
|---|---|---|---|---|
| −217.26 | −97.21 | 219.75 | −164.49 | −350.46 |

After changing home, rebuild `rv2aj_control` and `rv2aj_moveit_config` and restart.

### Collision model

The URDF has a 1.4 × 1.4 m table slab at the base mounting plane (Z = 0) and an 80 × 80 × 120 mm
collision box for the gripper. MoveIt refuses any pose where the arm or gripper touches the table.
Nothing else on the station is modelled.

### Frozen joints

`frozen_joints: [J5]` locks a joint in case of a hardware fault: straight moves keep it fixed and
let the tool pitch change instead, joint targets keep its current angle, and the driver refuses to
move it. The list is empty in normal use.

---

## 6. ROS 2 stack

### Packages

| Package | Content |
|---|---|
| `rv2aj_description` | URDF/xacro with limits and offsets from `joint_limits.yaml`, table, gripper box, meshes |
| `rv2aj_moveit_config` | SRDF (group `arm`, home state, disabled collision pairs), MoveIt launch |
| `rv2aj_interfaces` | messages, services, actions |
| `rv2aj_control` | nodes, libraries, config, launch, tests |

### Nodes

All nodes run in the `/rv2aj` namespace.

| Node | Role |
|---|---|
| `driver` | Owns the serial port. Polls joints, state and I/O; runs moves, gripper, outputs, servo and override. Publishes `status` and `joint_states`. Uses the simulated controller when `use_fake_hardware` is set. |
| `planner` | Checks every move with MoveIt before it reaches the driver (`planned/tcp_move`, `planned/joint_move`). Also serves MoveIt's FollowJointTrajectory controller. |
| `sequencer` | Teach points, loop playback, home and auto-run. |
| `plc_bridge` | Reads and writes DB10 on the S7-1200 with python-snap7. Idle while `plc_ip` is empty. |
| `translator` | Turns simple text commands ("move up 20", "go to point 2", "stop") into moves. |
| `voice` | Microphone input to `voice/text`; off by default. |
| `gui` | The control panel. |

`move_group` and `robot_state_publisher` run in the root namespace and follow `/rv2aj/joint_states`.

### Launch

`ros2 launch rv2aj_control bringup.launch.py`:

| Argument | Default | Meaning |
|---|---|---|
| `use_fake_hardware` | false | simulate the controller |
| `port` | /dev/ttyUSB0 | serial port |
| `use_moveit` | true | route moves through the planner |
| `gui` | true | control panel |
| `voice` | true | voice node (the start script turns it off) |
| `points_file` | auto | `~/.ros/rv2aj_points.json`, or `~/.ros/rv2aj_sim/points.json` in sim |
| `plc` | true | PLC bridge |

The simulator keeps its own points file, copied from the real one on first run, so testing never
touches the real points.

### Interfaces

Topics:
- `status` (`RobotStatus`): connection, servo, alarm, motion, override, joints, tool pose, last
  error, gripper, line I/O.
- `joint_states`, `sequencer/status`, `voice/text`, `plc/part_ready`, `plc/state` (JSON).

Driver services: `connect`, `disconnect`, `servo`, `stop`, `reset_alarm`, `set_override`,
`run_program` (untested), `gripper/open`, `gripper/close`, `line/set`.

Sequencer services: `points/save`, `points/delete`, `points/goto`, `points/clear`,
`points/set_speed`, `points/set_io`, `loop/start`, `loop/stop`, `loop/auto`, `home/goto`.

PLC bridge services: `plc/maintenance`, `plc/release_1..3`, `plc/line_stop`, `plc/line_reset`.

Actions: `tcp_move` and `joint_move` on the driver (unchecked), `planned/tcp_move` and
`planned/joint_move` on the planner.

```bash
ros2 topic echo --once /rv2aj/status
ros2 service call /rv2aj/servo std_srvs/srv/SetBool "{data: true}"
ros2 action send_goal /rv2aj/planned/tcp_move rv2aj_interfaces/action/TcpMove "{axis: Z, distance_mm: 10.0}"
ros2 action send_goal /rv2aj/planned/joint_move rv2aj_interfaces/action/JointMove "{relative: true, joint: J1, offset_deg: 5.0}"
```

### Libraries

| File | Content |
|---|---|
| `serial_link.py` | port handling, framing, command sequences, reply parsing |
| `fake_link.py` | controller simulator working on the real command strings |
| `driver_core.py` | driver logic: worker thread, polling, settle detection, safe shutdown |
| `kinematics.py` | 5-axis FK/IK, straight lines, limits, offsets, frozen joints |
| `plc_link.py` | PLC handshake logic, no ROS dependency |
| `voice_commands.py` | text command parser |
| `widgets.py` | panel widgets and theme |

### Configuration

`rv2aj_control/config/rv2aj.yaml`:

- `driver`: port, default override (10 %), max TCP move (50 mm), max joint step (45°), poll intervals.
- `planner`: check step (1°), line step (5 mm), line tolerance (1 mm), planning time.
- `sequencer`: points file, `home_joints_deg`, `home_speed_percent` (40), gripper wait,
  `auto_trigger` (`plc` or `input`), `auto_input_bit` (1), `auto_start_point` (`P2E`).
- `plc_bridge`: `plc_ip` (empty = off), port 102, rack 0, slot 1, DB 10, poll and reconnect times.

### Tests

```bash
cd rv2aj_ws/src/rv2aj_control && python3 -m pytest -q test/
```

They need the built and sourced workspace. `test/conftest.py` gives every test a generic robot
(wide limits, no offsets, nothing frozen) so the real calibration cannot leak into them.

---

## 7. How a move is executed

1. A panel button, the sequencer or the translator sends a request to the planner.
2. The planner builds the path:
   - Straight TCP move: IK every 5 mm along the line, keeping tool pitch and J6, reduced to the
     fewest waypoints that stay within 1 mm of the line. A joint jump over 30° between samples
     (a singularity) cancels the move.
   - Joint move: a straight line in joint space if it is collision-free, otherwise an OMPL plan
     shortened to the fewest valid waypoints.
3. Every waypoint, and the path between them at least every 1°, is checked against the software
   joint limits and by MoveIt (self-collision, table, gripper).
4. The driver runs each waypoint as an absolute joint move and polls until the arm settles:
   reached = at target and still for 0.2 s, stopped = still for 1 s away from target.
5. The result is reached, stopped short, or refused with a reason.

Limits per move: 50 mm for a TCP move, 45° for a joint step. Default speed is 10 %.

Since moves cannot be blended, there are no smooth arcs: a curve has to be sent as short straight
segments with a short stop between them.

### Playback

Each taught point stores its joints, gripper state, speed and a list of outputs to set.
For every move to point *i* the sequencer:

1. sets the speed to point *i*'s speed (the speed of a point is the speed of the move into it;
   Home uses `home_speed_percent`), re-sending the override only when it changes;
2. sends the point as one absolute joint move and waits for it to settle;
3. on arrival, sets the gripper (only if it changed) and writes the point's outputs.

Each point stops for about 0.4 s. That is the arm decelerating to a full stop, which cannot be
avoided without blending. The extra I/O reads are done only when the arm is idle so they don't add to it.

---

## 8. Control panel

The panel opens full screen (F11 toggles). The header shows connection, servo and alarm state, the
port, Connect/Disconnect and STOP. There are two tabs, Operation and Maintenance & safety.

### Operation tab

| Area | Function |
|---|---|
| Joints | live angle and −/+ step buttons per joint (1/5/10/20/45°), going through the planner |
| TCP | tool pose, program row, step distance (5/10/20/50 mm), Up/Down/Left/Right/Forward/Back |
| Gripper | Open / Close and the last commanded state |
| Robot IO | LEDs for IN0–15, OUT0–15 and the hand I/O 900–907 (pneumatic valves). Outputs can be clicked to toggle. With "Tıklananı seçili noktaya kaydet" checked, a clicked output is also stored on the selected point. |
| Motion | Servo ON/OFF, speed override, Home, Reset alarm |
| Points & Loop | save, insert after, go to, delete, clear; point table with speed; Slow (35 %) / Fast (60 %) for the selected point; Start/End loop; auto-run button |
| Event log | everything the stack reports |

Keys: `+` (also on the numpad) stops the robot even while typing in a field, Esc stops, F11 toggles full screen.

### Maintenance & safety tab

| Card | Function |
|---|---|
| Emergency stop | STOP ALL: robot stop, loop and auto-run off, servo off, and a latched line stop on the PLC (all stoppers up). The gripper is left alone so a held part doesn't fall. Stays active until Reset stop, which asks to confirm the area is clear. |
| Robot | Servo ON/OFF, Home, Reset alarm, gripper |
| Conveyor stoppers | PLC link state, maintenance mode, and for each station the car/parts/piston state and a Release button (one car through, maintenance mode only) |
| Robot I/O | read-only view of every I/O signal with names |

STOP ALL is a software stop over USB and the network. The hardware E-stop is the real safety stop.
The `+`, Esc and header STOP keys stop the robot only and don't touch the conveyor.

### Auto-run

The sequencer can run one pass through the taught points each time a part is ready. The trigger
is either the PLC (`auto_trigger: plc`) or the part sensor on the TCP, wired to IN1
(`auto_trigger: input`).

- The operator arms it from the panel. It needs the servo on and at least one taught point.
- A pass starts only when the arm is idle and parked at home (every joint within 10°) and the
  trigger goes high. It starts at the point after `auto_start_point` (P2E), runs to the last point
  and goes back home. There is no wrap-around.
- The sensor moves with the arm, so its readings during a pass are ignored. Returning home re-arms it.
- STOP disarms auto-run. After a restart it starts disarmed.

---

## 9. Transfer line and PLC link

### Station 4 I/O

| Signal | Robot I/O | Meaning |
|---|---|---|
| order bits 0/1 | IN6 / IN7 | 0 none, 1 assembly, 2 sort out, 3 take from pallet |
| error code bits 0/1 | OUT4 / OUT5 | error code 0–3 to the station |
| busy | OUT6 | working on an order |
| ready | OUT7 | station ready |
| — | OUT0–OUT3 | factory signals, never written |

The robot does not drive the conveyor. The driver reads the I/O bytes in its poll and writes one
bit at a time, only on request, reading it back to confirm.

### Conveyor PLC

The original Festo master controller is replaced by an S7-1200 that runs three stopper stations.
The PLC sits on a local router; the laptop reaches it over Wi-Fi.

Cars go round 1 → 2 → 3 → 1. Each station has a part sensor (sees all four parts on a car), a car
sensor and a stopper piston (valve on = piston down = car passes; it goes back up 1 s after the car has left).

- Station 1 (robot): holds a car with parts and asks for a robot pass, which picks all four parts.
  If parts are left it asks for one more pass, then releases the car and raises MissedPart on the panel.
  It never releases a car while the robot is busy.
- Station 2: empty cars wait to be filled by hand. A filled car leaves only when station 3 asks for one.
- Station 3: full cars wait, empty cars pass. One car is released whenever station 1 releases its car.

The PLC and the laptop share DB10 "PC_Link" (optimized access off, PUT/GET allowed). The PLC writes
part ready, heartbeat, pass count, station sensors and valve states; the laptop writes busy, done,
error, armed, heartbeat and the maintenance bits. The exact byte layout is in `plc/LineControl.scl`.
To build the blocks in TIA Portal, import the file under External source files and run Generate blocks.

Handshake: PartReady starts a pass, Busy clears PartReady, and the laptop reports Done (or Error if
the pass failed or was stopped). Both sides send a heartbeat with a 3 s timeout; while the link is
down no pass is started. The line stop is latched inside the PLC and clears only on a LineReset
from the laptop, so a lost connection or a restarted bridge can't restart the line.

To use it, set `plc_ip` in `rv2aj.yaml` and `auto_trigger: plc`.

Testing without the PLC:

```bash
python3 test/fake_plc.py [--stubborn]
ros2 run rv2aj_control plc_bridge --ros-args -r __ns:=/rv2aj -p plc_ip:=127.0.0.1 -p plc_port:=11020
```
