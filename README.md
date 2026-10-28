# RV-2AJ Robot Cell

Control software for a Mitsubishi MELFA RV-2AJ arm (CR1-571 controller) on Station 4 of a Festo
MPS 500 transfer line: a ROS 2 + MoveIt stack that drives the controller over RS-232, a PyQt
operator panel, and a link to the S7-1200 PLC that runs the conveyor.

<img src="docs/images/real_robot_assembly.gif" width="100%" alt="RV-2AJ real assembly">

<img src="docs/images/gui_control_panel.png" width="100%" alt="RV-2AJ control panel">

## Quick start

```bash
git clone git@github.com:reis0x01/RV-2AJ.git
cd RV-2AJ/rv2aj_ws
source /opt/ros/lyrical/setup.bash        # or jazzy
colcon build && source install/setup.bash
cd src/rv2aj_control && python3 -m pytest -q test/
```

Needs ROS 2 with MoveIt 2, plus `python3-pyqt5`, `python3-scipy`, `python3-serial` and
`python-snap7`. Teach points live in `~/.ros/rv2aj_points.json`; copy one from `points_backups/`.

```bash
./start_robot.sh          # real robot on /dev/ttyUSB0
./start_robot.sh sim      # simulated controller
./stop_robot.sh           # always stop this way
```

`stop_robot.sh` lets the driver send STOP, SRVOFF and CNTLOFF before shutting down. The log is in
`~/.ros/rv2aj_stack.log`.

In the panel: wait for CONNECTED, press Servo ON (only ever done by a person), then jog, go Home or
run points. `+`, Esc, the STOP button or the hardware E-stop stop the arm. After an alarm press
Reset alarm; the driver cycles the servo, since the first move after a reset fails otherwise.

## Hardware

- **Arm:** 5 axes (J1, J2, J3, J5, J6; there is no J4). Shoulder 300 mm above the base, upper arm
  250 mm, forearm 160 mm, wrist to flange 72 mm. The base is mounted turned 180°.
- **Controller:** CR1-571, commands sent as EXEC over RS-232. Its MEJAR joint limits are the final
  authority. OUT0–OUT3 are factory signals and are never written.
- **Gripper:** Festo HGP-16, pneumatic, `EXECHOPEN 1` / `EXECHCLOSE 1`.
- **Serial:** FTDI USB adapter, 9600 baud, 8E2. User must be in `dialout`.
- **Cell:** only the table is in the collision model. The block, chute and rails around the base are not.

## Layout

```
rv2aj_ws/src/
  rv2aj_description/     URDF, meshes, config/joint_limits.yaml (limits, zero offsets)
  rv2aj_moveit_config/   SRDF and MoveIt launch
  rv2aj_interfaces/      messages, services, actions
  rv2aj_control/         nodes, config/rv2aj.yaml, launch, tests
plc/LineControl.scl      S7-1200 program for the conveyor
docu/                    write-ups
points_backups/          teach-point snapshots
```

After changing `joint_limits.yaml` or the home pose in `rv2aj.yaml`, rebuild and restart.

## Serial protocol

Commands are sent as `1;1;<COMMAND>\r`. A reply starting with `Qo` is OK, `Qe`/`Qx` is an error,
and a lowercase status (`Qok`) means an alarm is pending.

| Purpose | Command |
|---|---|
| Read joints / pose / state | `JPOSF`, `PPOSF`, `STATE` |
| Read I/O | `VALM_INB(0)`, `VALM_OUT(n)` |
| Operation rights, servo | `CNTLON`, `SRVON` / `SRVOFF` |
| Stop, reset | `STOP`, `RSTALRM` |
| Speed | `EXECJOVRD <1-100>` |
| Joint move | `EXECJTMP=J_CURR`, `EXECJTMP.J1=JTMP.J1+RAD(5)`, `EXECMOV JTMP` |
| Output | `EXECM_OUT(n)=1` |

- `EXECMOV` returns at once; the driver polls `JPOSF` until the arm settles.
- A second `EXECMOV` during a move is refused (error 5640), so moves can't be blended and the arm
  stops briefly at every point.
- Tuple expressions like `J_CURR+(…)` are a syntax error; absolute moves are built joint by joint.
- The error in the 5th field of `STATE` is the last one recorded, not necessarily an active one.
  Use `VALM_ERRNO` instead.

## ROS nodes

All nodes run under `/rv2aj`.

| Node | Role |
|---|---|
| `driver` | owns the serial port, polls status, runs moves, gripper and outputs |
| `planner` | checks every move with MoveIt (self-collision, table) before the driver gets it |
| `sequencer` | teach points, loop playback, home, auto-run |
| `plc_bridge` | exchanges DB10 with the PLC; idle while `plc_ip` is empty |
| `translator` | text commands such as "move up 20" |
| `gui` | operator panel |

Straight TCP moves are solved with IK every 5 mm and split into waypoints that stay within 1 mm of
the line. Joint moves use a straight joint-space path, or OMPL if that collides.

Software joint limits are currently off (±720° in `joint_limits.yaml`); the measured limits are
kept there as comments.

```bash
ros2 topic echo --once /rv2aj/status
ros2 action send_goal /rv2aj/planned/tcp_move rv2aj_interfaces/action/TcpMove "{axis: Z, distance_mm: 10.0}"
```

## Operator panel

- **Operation tab:** joint jog, TCP moves, gripper, all robot I/O (outputs clickable), servo and
  speed, teach points with a Slow (35 %) / Fast (60 %) speed per point, loop and auto-run.
- **Maintenance & safety tab:** STOP ALL (robot stop, servo off, auto-run off and a latched
  conveyor stop), robot controls, manual stopper release, and a read-only view of every signal.

STOP ALL is a software stop. The hardware E-stop is the safety stop.

**Auto-run:** when armed, the sequencer runs one pass through the points each time a part is ready,
signalled by the PLC or by the part sensor on IN1. A pass only starts with the arm parked at home
(point P2E) and runs from the next point to the end, then back home.

## Conveyor PLC

An S7-1200 replaces the original Festo controller and runs three stopper stations. Cars go
1 → 2 → 3 → 1:

- **Station 1 (robot):** holds a full car until the robot has taken all four parts.
- **Station 2:** empty cars are filled by hand and sent on when station 3 asks.
- **Station 3:** holds full cars and lets one go each time station 1 releases its car.

The PLC and the laptop share DB10 (byte layout in `plc/LineControl.scl`) with a heartbeat on both
sides. Set `plc_ip` in `rv2aj.yaml` to enable it. To test without the PLC:

```bash
python3 test/fake_plc.py
ros2 run rv2aj_control plc_bridge --ros-args -r __ns:=/rv2aj -p plc_ip:=127.0.0.1 -p plc_port:=11020
```
