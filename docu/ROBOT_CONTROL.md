# Control of the Mitsubishi RV-2AJ: Methods, Constraints and Findings

A detailed account of the control approaches applied to a Mitsubishi RV-2AJ
manipulator with a CR1-571 controller, intended as source material for
publication.

Specifications are cited from Mitsubishi's own documentation. Behaviour marked
**OBSERVED** was measured on the physical system. Anything not verified is
marked **UNVERIFIED** and should be checked before being published as fact.

---

## Table of contents

1. [Platform description](#1-platform-description)
2. [The five-degree-of-freedom constraint](#2-the-five-degree-of-freedom-constraint)
3. [Method A: manual teaching via the pendant](#3-method-a-manual-teaching-via-the-pendant)
4. [Method B: host control over RS-232](#4-method-b-host-control-over-rs-232)
5. [The command-level interface problem](#5-the-command-level-interface-problem)
6. [Method C: resident program with host triggering](#6-method-c-resident-program-with-host-triggering)
7. [Continuous-path motion](#7-continuous-path-motion)
8. [The implemented system: teach-and-repeat with per-point parameters and sensor-triggered automation](#8-the-implemented-system-teach-and-repeat-with-per-point-parameters-and-sensor-triggered-automation)
9. [Comparison of methods](#9-comparison-of-methods)
10. [Notes for publication](#10-notes-for-publication)

---

## 1. Platform description

### 1.1 Manipulator specifications

All values from the **Mitsubishi RV-1A / RV-2AJ Standard Specifications
Manual**, Table 2-1. **CONFIRMED.**

| Property | Value |
|---|---|
| Degrees of freedom | **5** |
| Axes present | J1, J2, J3, J5, J6 (**no J4**) |
| Maximum payload | 2 kg |
| Rated payload | 1.5 kg |
| Maximum reach | 410 mm |
| Pose repeatability | **±0.02 mm** |
| Mass | approx. 17 kg |
| Ambient temperature | 0 to 40 °C |

Maximum payload applies with the flange facing downward within ±10 degrees.

### 1.2 Joint operating ranges and maximum speeds

**CONFIRMED** from the same source. These are the values any motion check should
clamp against.

| Joint | Function | Range (deg) | Span | Max speed (deg/s) |
|---|---|---|---|---|
| **J1** | Waist rotation | −150 to +150 | 300 | 180 |
| **J2** | Shoulder | −60 to +120 | 180 | 90 |
| **J3** | Elbow | −110 to +120 | 230 | 135 |
| **J4** | **does not exist** | | | |
| **J5** | Wrist pitch | −90 to +90 | 180 | 180 |
| **J6** | Wrist roll | −200 to +200 | 400 | 210 |

Note that the controller retains the **J4** label in its numbering even though
the axis is absent. Software written for a 6-axis MELFA will index wrongly on
this arm if that is not handled.

### 1.3 Controller and interfaces

| Item | Value |
|---|---|
| Controller | CR1-571 |
| Language | MELFA-BASIC IV |
| Teaching pendant | R28TB |
| Host interface | RS-232C, front panel connector |
| Fieldbus to cell | SysLink, 24-pin, to `XPA4` |
| Ethernet | **none on this controller** |

The absence of Ethernet is a defining constraint of this platform and shapes
everything in sections 4 to 6.

### 1.4 Measured joint ranges on this specific unit

The ranges in section 1.2 are the manufacturer's specification. The **actual
software limits in use** were obtained empirically on this unit with a probe
script (`test/probe_joint_limits.py`): 5-degree steps at 5 % speed until the
controller raised a range alarm, with the limit then set about 2 degrees inside
the alarm point. **OBSERVED**, values are controller readings in the current
(software-calibrated, section 5.1) frame, not the manual's frame:

| Joint | Alarm points (measured) | Limit set | Note |
|---|---|---|---|
| J1 | −329.7 / −19.5 | −327.5 … −21.5 | + end repeated at −19.46 |
| J2 | −152 (table) / +2.7 | −150 … +0.5 | − side stopped by the table guard, not the controller |
| J3 | −44.7 / +181.6 | −43 … +210 | raised 2026-10-16: the controller soft limit (`MEJAR`) was found wide open (±1800°); the arm was jogged to 194.41 |
| J5 | −274 / −82.0 | −272 … −84 | ≈192° span; the + end at −82 is where it "refused to move up" |
| J6 | none in −425 … +339 | −425 … +335 | no alarm in the tested range |

Two points are worth stating for a reader. First, these readings are in a frame
the operator re-calibrated in software (section 5.1), so they do not match the
manual's symmetric ranges numerically even though the physical travel does.
Second, the controller's own range parameter (`MEJAR`) was found effectively
disabled on this unit, so the *controller* did not enforce J3 until ±1800°; the
enforced limits are therefore the ones the host stack applies, which is itself a
finding about the state an obsolete unit can be left in.

---

## 2. The five-degree-of-freedom constraint

This is the most consequential property of the platform and deserves explicit
treatment in any paper.

A general rigid-body pose in three dimensions requires **six** degrees of
freedom: three for position, three for orientation. The RV-2AJ has **five**.
The missing axis is J4, the wrist yaw.

The consequence is that the manipulator **cannot reach an arbitrary pose**.
For any reachable position, the attainable orientations form a one-parameter
restricted set rather than the full SO(3). In practice:

- The approach vector is constrained relative to the arm's configuration
- Top-down picking is natural; arbitrary-angle picking is not
- Grasp planning must either fix the approach direction or solve a reduced
  inverse-kinematics problem that accepts orientation error

For a vision-guided picking application this is not a minor detail. A pose
estimate from a camera delivers six parameters, and two of them cannot be
realised independently. Any system built on this arm must either:

1. Constrain the task so that the lost degree of freedom does not matter, for
   example by restricting to planar objects picked from above, or
2. Project the desired pose onto the reachable manifold and accept the
   resulting orientation error, or
3. Exploit object symmetry, which is why cubes and cylinders are forgiving
   targets and arbitrary parts are not

This repository's tidying task was scoped to small cubes for exactly this
reason.

---

## 3. Method A: manual teaching via the pendant

The classical industrial approach, often termed **online programming** or
**lead-through teaching** in the literature.

### 3.1 Procedure

1. Controller to **TEACH** mode, pendant **enabled**
2. Select or create a program number
3. Jog the manipulator to the desired pose using the pendant's axis keys, in
   either joint or Cartesian jog mode
4. Store the pose into a position variable
5. Repeat for each waypoint
6. Write the motion program referencing those variables
7. Verify by stepping through the program at reduced speed
8. Controller to **AUTO**, run

### 3.2 R28TB key combinations

Several widely circulated instructions are **wrong for this pendant**. The
following are **CONFIRMED** from the manual and by operation:

| Action | Keys |
|---|---|
| Decrease speed | `STEP/MOVE` + `BACKWD` |
| **Increase speed** | `STEP/MOVE` + `FORWD` |
| **Store a position** | `STEP/MOVE` + `ADD`, held **together** |
| Step forward through a program | `+/FORWD` + `INP/EXE` |

The R28TB has **no dedicated OVRD up/down keys**. Any procedure referencing
them belongs to a different pendant generation.

Two errors arising from this were observed and corrected during the work:
position storage attempted as `ADD` then `INP/EXE` (which does not store), and
an attempt to reduce speed with `STEP/MOVE` + `FORWD`, which increases it.

### 3.3 Characteristics

**Advantages**

- No external hardware or software required
- The taught pose is the true achieved pose, so kinematic model error,
  calibration error and gravitational deflection are all absorbed implicitly
- Repeatability of ±0.02 mm applies directly to taught points
- Safe: the operator is present with the enabling device and can see the arm

**Limitations**

- Points are **static**. The program cannot respond to a part in a new place
- Teaching time scales linearly with the number of waypoints
- The arm must be taken out of production to reteach
- No access to external sensing, computation or decision-making
- Position data lives only in the controller unless deliberately backed up

### 3.4 The backup-battery dependency

Taught positions, program data, parameters **and the origin** are held in
battery-backed memory. When the controller's backup batteries failed, all of
this was at risk and the origin was lost, producing joint readings of
approximately 22676 degrees.

**Finding, CONFIRMED against Mitsubishi's troubleshooting documentation:**
re-entering the factory **DATA** origin codes does **not** restore the origin
after battery loss. The **ABS** method, using the physical alignment marks on
the arm, is the documented recovery procedure. The DATA codes were entered
correctly several times with no effect before this was established.

This is a reproducible failure mode of the platform and worth reporting. The
mean time to failure of the backup batteries is far shorter than the service
life of the manipulator, and the recovery procedure is not the one most
operators attempt first.

---

## 4. Method B: host control over RS-232

### 4.1 Physical and link layer

**CONFIRMED** by establishing a working link.

| Parameter | Value |
|---|---|
| Connector | RS-232C, front of the CR1-571 |
| Host device | `/dev/ttyUSB0` via a USB-serial adapter |
| Baud | 9600 |
| Data bits | 8 |
| Parity | **Even** |
| Stop bits | **2** |
| Flow control | none |

Controller-side parameters:

| Parameter | Value |
|---|---|
| `COMDEV` | `RS232` |
| `CPRCE11` | `0` |

Preconditions: the pendant must be **disabled**, and the controller must be in
**AUTO (Ext)**.

### 4.2 Frame format

```
1;1;<COMMAND>\r
```

The two leading fields are the robot number and the slot number, terminated by a
carriage return. Responses begin with `Q` and a status word:

| Response | Meaning |
|---|---|
| `Qo…` | Command accepted; any payload (e.g. a variable value) follows |
| `Qe…` / `Qx…` | Error / rejected |

**OBSERVED, and important in practice:** a **lowercase** status letter
(`Qok`, `Qer`) signals that an **alarm is pending** even when the command itself
was accepted; the alarm is cleared with `RSTALRM`. The host must therefore not
treat a syntactically successful reply as "all well" without also checking the
alarm state reported by `STATE` (section 4.7). A further quirk of this unit
(**OBSERVED**): after any alarm reset, the *next* motion command fails unless the
servo is cycled **off and on** again, so the driver's reset routine performs
`SRVOFF`/`SRVON` automatically.

### 4.3 Confirmed command set

Every command below was verified by readback on the physical hardware.
**OBSERVED.** Each is sent inside the `1;1;…` frame of section 4.2.

| Command | Function | Class |
|---|---|---|
| `CNTLON` | Acquire control (operation) rights | Session |
| `CNTLOFF` | Release control rights | Session |
| `SRVON` | Servo on | Power |
| `SRVOFF` | Servo off | Power |
| `STOP` | Halt; **also halts an `EXECMOV` in progress** | Motion |
| `RSTALRM` | Reset a latched alarm | Recovery |
| `STATE` | Read controller state word (mode, alarm, error field) | Read-only |
| `PPOSF` | Read Cartesian pose (X, Y, Z, A, B…) | Read-only |
| `JPOSF` | Read joint angles J1…J6 (J4 reports `****`) | Read-only |
| `VAL<expr>` | Read the value of a variable or expression | Read-only |
| `EXECJOVRD <n>` | Set joint speed override, 1–100 % | Motion parameter |
| `EXECJTMP=J_CURR` | Copy the current joint pose into `JTMP` | Motion build |
| `EXECJTMP.J<n>=JTMP.J<n>+RAD(<d>)` | Add `d` degrees to one joint of `JTMP` | Motion build |
| `VALJTMP` | Read `JTMP` back (for verification before moving) | Read-only |
| `EXECMOV JTMP` | Move to the joint pose held in `JTMP` | Motion |
| `EXEC…HAND…` (gripper, with hand number) | Open/close the pneumatic hand | I/O write |
| `EXECM_OUT(n)=1` | Set digital output bit n | I/O write |
| `VALM_INB(b)` | Read digital input byte b (e.g. `VALM_INB(0)`) | I/O read |
| `VALM_OUTB(b)` | Read digital output byte b | I/O read |

Two prefixes are visible in this set and the distinction matters:

- **`EXEC`** submits a single MELFA-BASIC line for **immediate execution**
- **`VAL`** reads the value of a variable or expression

`EXECJOVRD` requires control rights: without a preceding `CNTLON` it is rejected
with **L6020 "operation is disabled"** (**OBSERVED**).

### 4.3.1 The verified absolute-joint-move sequence

There is no single "move to these joint angles" command exposed cleanly over
this interface; it must be **built** on the controller through a temporary
variable. The sequence below is the one verified on this unit, and the stack
uses no other form for absolute joint motion:

```
EXECJOVRD <percent>              set speed first (takes effect on the next EXECMOV)
EXECJTMP=J_CURR                  JTMP := current pose
VALJTMP                          read JTMP back (sanity)
EXECJTMP.Jn=JTMP.Jn+RAD(<delta>) for each moving joint n, delta = target − current, in degrees
VALJTMP                          read JTMP back and verify every component before moving
EXECMOV JTMP                     execute
```

`EXECMOV` replies as soon as motion **starts**; arrival is detected by polling
`JPOSF` (section 4.7), not from the reply.

**Rejected forms, OBSERVED.** The compact tuple expression
`J_CURR+(d1,d2,…)` is **not** accepted and raises **L4220 (syntax error)**.
This is why the stack edits one joint component at a time and reads `JTMP` back
before issuing `EXECMOV`: a wrong guess here is a motion command, not a compile
error (the discipline of section 4.5). This exact invariant — that only the
verified `JTMP`/`EXECMOV` form is ever emitted and never the tuple form — is
pinned by an automated test
(`test_absolute_joint_move_uses_only_verified_syntax`).

### 4.4 The control-rights model

The controller enforces exclusive control. `CNTLON` acquires it; while held,
the pendant cannot command motion, and vice versa. This is a safety
interlock, not a software limitation, and it means a host program and a human
operator cannot both drive the arm simultaneously.

### 4.5 Methodological discipline applied

MELFA-BASIC IV has **no JOG instruction**. This was confirmed from the manual
after repeated requests to implement continuous jogging from a graphical
interface.

Rather than guess at motion syntax, an **empirical syntax-finder** was written
with the following safety properties:

- Dry-run by default; motion requires an explicit `--arm` flag
- Forces `EXECJOVRD 5`, i.e. 5 percent of maximum speed
- Restricts candidate motions to a 2 degree step on **J1 only**
- Pauses for operator confirmation before each candidate command
- Logs the controller's response to every attempt

The principle generalises: on a PLC an incorrect guess produces a compile
error, on a manipulator it produces a collision. Any published methodology for
reverse-engineering a legacy robot interface should state its containment
measures explicitly.

### 4.6 Error and alarm codes observed

The following codes were seen on this unit and are the ones a host program must
recognise and handle. **OBSERVED.** The `STATE` error field reports the most
recent error, which is not necessarily an active one.

| Code | Meaning | Context in which it appeared |
|---|---|---|
| 0965 | Excessive error 1 | servo could not follow the commanded motion |
| 1060 / 1069 | Encoder communication error | appeared as the stale "last error" in `STATE` |
| 1120 | Encoder ABS position data lost | origin lost after battery failure (section 3.4) |
| 4220 (L4220) | Syntax error | e.g. the rejected `J_CURR+(…)` tuple form (section 4.3.1) |
| 5150 | The origin has not been set | origin lost — **every `EXEC` command is then refused** |
| 5600 | Cannot execute during an error | an alarm must be reset first |
| 5640 | Cannot execute during RUN | a motion is still running — see section 5 |
| 6010 | Illegal command | |
| 6020 (L6020) | Operation is disabled | `CNTLON` was missing before `EXECJOVRD` |
| `81069x` in `STATE` | joint range limit on axis x | a controller range limit (`MEJAR`) reached |

Error 5150 is the one that makes the battery/origin failure of section 3.4 not
merely a calibration nuisance but a total loss of host control: with the origin
unset, *no* `EXEC` command runs at all until the origin is recovered by the
`ABS` mark method.

### 4.7 Polling cadence

The host reads state continuously so that motion completion, alarms and I/O are
observed without blocking. The cadence used on this link, chosen against the
≈873 character/second throughput of section 5.4:

| Query | Interval | Rationale |
|---|---|---|
| `JPOSF` | every cycle, ≈0.03 s loop (one query ≈0.1 s at 9600 baud) | arrival detection needs responsive joint readings |
| `STATE` + `PPOSF` | 1.0 s idle, 0.5 s while moving | mode/alarm/pose, faster while a move is tracked |
| Line I/O bytes (`VALM_INB`/`VALM_OUTB`) | all four higher bytes at once when idle; one byte round-robin while moving; skipped when a command is queued | keeps a queued move from waiting behind four reads (section 8.2) |

Arrival is declared from `JPOSF`: a pose is **reached** when it is at target and
unchanged for 0.2 s, **stopped short** when it is unchanged for 1.0 s away from
target, with a 120 s timeout. These thresholds are parameters of the driver, not
of the controller.

---

## 5. The command-level interface problem

This is the central technical finding of the work and the part most likely to
be of interest to a reader.

### 5.1 Observation

When motion commands are issued individually over RS-232, the manipulator
**decelerates to zero velocity at every waypoint**, regardless of speed and
acceleration settings. Attempting to issue a second motion command while the
first is executing returns:

```
error 5640: Cannot execute during RUN
```

### 5.2 Explanation

`EXEC` submits **one line** for immediate execution. The controller's
trajectory planner receives a single target and has no knowledge of any
subsequent target. Corner blending requires the planner to know the next
waypoint **while still in motion toward the current one**, in order to compute
the departure from the nominal path. With a one-command-at-a-time interface
this information does not exist at planning time.

The deceleration to zero is therefore not a tuning deficiency. It is a
**necessary consequence of the interface granularity**.

### 5.3 Generalisation

This is worth stating in general terms because it applies well beyond this
platform:

> An external interface that exposes **commands** rather than **trajectories**
> imposes a hard lower bound on cycle time equal to the sum of the
> accelerate-decelerate transients at every waypoint. No amount of parameter
> tuning on the host side can remove it.

Legacy industrial controllers commonly expose exactly this kind of interface.
Modern alternatives, by contrast, accept a trajectory or a stream of setpoints
at a fixed control rate, which is what makes visual servoing and
sensor-reactive motion feasible.

### 5.4 Compounding factor: serial latency

At 9600 baud, 8E2, each character consumes 11 bit periods, so throughput is
approximately 873 characters per second. A short command plus its response
costs on the order of tens of milliseconds, before any controller processing.
A host that waits for completion before issuing the next command therefore
adds this latency to **every** waypoint, on top of the mechanical transient.

**UNVERIFIED:** the per-waypoint overhead was not measured quantitatively.
A paper making a performance claim should instrument this directly, for example
by logging host-side timestamps across a fixed multi-waypoint path under each
control method and reporting the distribution.

---

## 6. Method C: resident program with host triggering

### 6.1 Principle

Invert the division of labour. Rather than the host issuing motion, the
**trajectory resides in the controller** and the host issues only high-level
commands.

```
Method B:   Host ──> Mov P1 ──> (wait) ──> Mov P2 ──> (wait) ──> ...

Method C:   Host ──> "run program 1"
            Controller, autonomously:  Cnt 1 ; Mov P1 ; Mov P2 ; Mov P3 ...
```

The controller's planner now sees the whole sequence and can blend.

### 6.2 For static paths

Write the program on the pendant, store it, and trigger it from the host. This
requires **no new serial syntax** beyond a program-run command and is the
fastest route to a working result.

### 6.3 For dynamic paths

Where waypoints are computed at runtime, for instance from vision, a
**producer-consumer buffer** is appropriate:

1. A resident program loops, reading waypoints from position variables
2. The host writes the next waypoints into those variables **ahead of** the
   manipulator reaching them
3. The planner always has lookahead, so motion stays continuous

The host must stay one or two waypoints ahead. The buffer depth needed is a
function of serial latency and of the time taken to traverse one segment, and
is a measurable quantity worth reporting.

### 6.4 Prerequisite, not yet established

The serial commands to **upload** and **run** a stored program, and to **write
a position variable**, have not been confirmed. **These must not be guessed.**
Two acceptable routes:

- Read them from the CR1 Controller Instruction Manual, communication section
- Discover them empirically using the same containment discipline as section
  4.5: read-only commands first, servo off, no motion until syntax is
  understood

---

## 7. Continuous-path motion

### 7.1 The CNT instruction

**CONFIRMED** syntax from the MELFA-BASIC IV command list:

```basic
CNT 1,100,200
```

The command list describes it as designating **smooth operation**. `CNT 1`
enables continuous path; `CNT 0` disables it.

**UNVERIFIED:** the meaning of the two numeric arguments. They are widely
understood to be deceleration-start and acceleration-end distances in
millimetres, but this was not confirmed from an authoritative source during
this work. **Do not publish a definition of these arguments without checking
the CR1 instruction manual.** Begin with `Cnt 1` alone.

### 7.2 Related motion parameters

**CONFIRMED** syntax:

| Instruction | Example | Effect |
|---|---|---|
| `OVRD` | `OVRD 100` | Speed override, percent, in 0.1 percent units |
| `ACCEL` | `ACCEL 50,80` | Acceleration and deceleration time as a percentage of maximum, in 1 percent units |
| `SPD` | `SPD 123.5` | Linear and circular interpolation speed, mm/s |
| `FINE` | `FINE 200` | Positioning completion condition, in encoder pulses |

`FINE` is the parameter controlling how tightly the controller verifies
arrival before proceeding. It is the mechanism underlying the per-waypoint
dwell, and relaxing it is an alternative to `CNT` where approximate positioning
is acceptable.

The syntax above is confirmed from the MELFA-BASIC IV command list, but a
distinction matters for this platform: **`OVRD` (as `EXECJOVRD`) is the only one
of these the implemented system issues over the serial link**, because that is
the one verified accepted here (section 4.3). Acceleration shaping via `ACCEL`
was **not** adopted over this interface and the implemented system deliberately
varies speed only (section 8.1). `CNT`, `FINE` and `SPD` belong to the
resident-program route (Method C, section 6), which was not implemented, so
their behaviour on this unit is **UNVERIFIED** here.

### 7.3 Program structure

```basic
10 Ovrd 100
20 Accel 100,100
30 Cnt 1
40 Mov P1
50 Mov P2
60 Mov P3
70 Cnt 0
80 Mov P4
90 End
```

### 7.4 The accuracy trade-off

With `Cnt 1` the manipulator **rounds the corner** and does not pass through
the intermediate waypoints exactly. This is the defining trade-off of
continuous-path operation and must be stated whenever a cycle-time improvement
is claimed.

The practical pattern, used above, is selective: `Cnt 1` for transit segments
where only the endpoints matter, `Cnt 0` before any pick or place pose where
the ±0.02 mm repeatability is the point of using this arm at all.

A paper reporting a speed improvement from `Cnt` should report the
corresponding **path deviation**, measured rather than assumed, otherwise the
result is not meaningful.

---

## 8. The implemented system: teach-and-repeat with per-point parameters and sensor-triggered automation

The system actually built is Method B productionised: a host-side
teach-and-repeat sequencer running on the laptop and driving the arm
point-to-point over RS-232. It accepts the per-waypoint dwell that section 9
marks as unavoidable for Method B, and engineers away only the part of that
dwell that is **not** physical.

### 8.1 Teach-and-repeat over RS-232

Each taught point is a record with a **name**, the six **joint angles** (J4
stored as 0, section 1.1), a **gripper** state (`open`/`closed`/none), a
per-point **speed** percentage and a list of per-point **outputs** (each an
`(output, value)` pair). Points are persisted as JSON (`~/.ros/rv2aj_points.json`).
Playback issues each point as the absolute joint move verified in section 4.3.1
(`EXECJTMP=J_CURR`, per-joint `EXECJTMP.Jn=JTMP.Jn+RAD(delta)`, `VALJTMP`
read-back, `EXECMOV JTMP`) and declares arrival by polling `JPOSF` against the
0.2 s / 1.0 s / 120 s thresholds of section 4.7 before continuing.

- **Per-point speed.** *(Since 2026-10-27 the sequencer uses the destination's speed only — speed per
  move; the slowest-of-two rule below is the earlier design.)* The speed override for a transition is the **slower of
  the two endpoints' speeds** — a "slowest-of-two" rule — so marking a grab
  point slow also slows the approach to it and the retreat from it. Concretely,
  the destination's speed and the last point actually reached are compared and
  `min` is taken; if nothing has been reached yet (or the previous move did not
  finish cleanly) only the destination's own speed is used. The GUI exposes two
  presets, **Slow = 35 %** and **Fast = 60 %** (10 % and 40 % until 2026-10-27); points saved before the feature
  existed default to **20 %**. **Home** has no taught speed of its own and is
  assigned a fixed **40 %** for the purpose of this rule. Only the override is
  varied — `ACCEL` is not issued (section 7.2).
- **Override is set before the move, and skipped when unchanged.** `EXECJOVRD`
  takes effect on the next `EXECMOV`, so the speed is sent first and the move is
  issued only once it is confirmed. When the required override already equals the
  controller's current override (authoritative in the status stream), the extra
  serial round-trip is skipped — most consecutive points share a speed, and
  re-sending it would just be a round-trip of standing still.
- **Per-point outputs and gripper actuated on arrival.** On reaching a point the
  sequencer runs its gripper command (if any) and then each saved output action
  in order, each waited out before the next; the loop only advances once all of
  them complete. The gripper is actuated **only when its state actually changes**
  (re-commanding it to where it already is would cost a command plus the
  `gripper_wait_s` = 0.6 s pneumatic settle for no physical effect). Outputs are
  written as single bits (driver `line/set`, one bit written and its byte read
  back), so a taught sequence drives the station's pneumatics and signals, not
  only the arm.

### 8.2 Reducing the per-waypoint dead time

The dead time first observed at each point (≈1.4 s) came not from motion but
from **host-side redundancy**, three contributions each costing serial
round-trips while the arm stood still:

1. **I/O re-reads.** Four higher I/O bytes (inputs 8–15, outputs 8–15, and the
   two hand bytes at 900) were being read at *every* point. These now refresh
   all at once only when the arm is idle, and round-robin **one byte per poll**
   while moving, backing off entirely when a motion command is already queued so
   a move is never delayed behind four reads.
2. **Redundant gripper command.** The gripper was re-commanded at every point
   even when its state was unchanged, each time incurring the 0.6 s pneumatic
   settle (section 8.1). It is now actuated only on a state change.
3. **Redundant override.** The speed override was re-sent at every point; it is
   now skipped when already correct (section 8.1).

Removing these cut the inter-point dead time to ≈0.4 s. The residual is the
**physical deceleration to a full stop** at each point, which cannot be removed
without `CNT` blending; section 5 and error 5640 establish that blending is
impossible over a one-command-at-a-time link. This is the honest ceiling: the
dwell is **reduced, not eliminated**. The ≈1.4 s → ≈0.4 s figures are coarse
operator observations, not instrumented measurements (section 10.2). The I/O
poll behaviour — all-at-once when idle, one byte while moving, skipped when a job
is queued — is pinned by an automated test
(`test_higher_io_poll_instant_when_idle_rotating_when_busy`).

### 8.3 Sensor-triggered single-pass automation

A part sensor mounted on the robot's **tool** and wired to digital input **IN1**
(`auto_input_bit = 1`) lets the cell run one assembly pass per arriving part.
Because the sensor rides with the arm, a reading is only meaningful at the park
pose, so the trigger is gated on the arm actually being there.

**At-home test.** "At home" is defined as every *fitted* joint (J1, J2, J3, J5,
J6 — J4 is skipped, section 1.1) being within `AUTO_HOME_TOL_DEG = 10°` of the
stored home pose, a NaN reading failing the test. The home pose used is the
operator's 2026-10-21 pick/wait pose
`[−217.26, −97.21, 219.75, —, −164.49, −350.46]`, which coincides with the
taught point **P2E**.

**Trigger.** On every status update, the sequencer forms
`ready = idle ∧ sensor ∧ at-home ∧ connected ∧ servo_on`, where `idle` means no
loop is running and no goal is outstanding and `sensor` is bit 1 of the input
byte. A pass is started on the **rising edge** of `ready` (it was false on the
previous update and is true now). This has two consequences without any explicit
queue: readings taken mid-pass, while the sensor is swinging through the air near
the stations, are ignored because the arm is not idle-and-at-home; and returning
home at the end of a pass re-establishes the condition, so a succession of parts
is served **one pass at a time**.

**Pass extent.** A pass does not restart from the first taught point. Because the
arm is already standing at the park point (P2E), the pass starts at the point
**immediately after** `auto_start_point` and runs to the last taught point; it
does not wrap into a continuous loop. If `auto_start_point` is not found among
the points, it falls back to starting at the first point (logged as a warning).
On completion the arm is sent back to the home pose and the automation re-arms
from there.

**Interface.** Arming/disarming is a service (`loop/auto`, `SetAutoRun{enable,
input_bit}`), and the sequencer publishes `auto_mode` / `auto_input_bit` in its
latched status so the GUI can reflect the armed/running state.

**Where the reactivity lives.** This is the sensor-reactive behaviour the
comparison table (section 9) marks "per-waypoint only" for Method B, obtained
here at the **pass level** — one trigger per pass — rather than inside the motion
loop. That is exactly what the serial latency of section 5.4 permits: the host
is not in the loop at the waypoint rate, only at the pass boundary.

**Safety invariants (OBSERVED, enforced in code):**

- Servo power is enabled **only by a human**; the automation never issues
  `SRVON`. Arming is refused unless the robot is connected, the servo is on, and
  at least one point is taught.
- **STOP disarms** the automation (clears `auto_mode`), so the physical/`+`-key
  stop is also an automation kill switch.
- The automation **starts disarmed** on every restart; a part already sitting at
  home when it is armed counts as a fresh trigger (the rising-edge state is reset
  on arming).

### 8.3.1 Experimental instance

The system was exercised on a real assembly task on Station 4 of the Festo MPS
500 line: a taught sequence of **47 points** carrying per-point speeds and
gripper/output actions, executed on the physical robot over RS-232 (the run is
recorded in the repository). This is the concrete instance behind the method; it
is an existence demonstration, not an instrumented benchmark (section 10.2).

### 8.4 Relationship to the methods

This is Method B applied with discipline: static teaching for the path, dynamic
host control for sequencing, per-point parameters for speed and I/O, and a
single external sensor for part-level reactivity. It does **not** reach Method
C's buffered continuous motion, which would require the program-upload path of
section 6.4 that has not been established. What it does demonstrate is that a
legacy, serial-only controller can still support **parameterised,
sensor-triggered automation**, provided the reactivity is placed at the pass
level rather than inside the motion loop.

---

## 9. Comparison of methods

| Property | A: Pendant | B: Serial per-command | C: Resident + trigger |
|---|---|---|---|
| External hardware | none | serial adapter | serial adapter |
| Waypoints | static | dynamic | static or dynamic |
| Continuous path | available | **impossible** | available |
| Per-waypoint dwell | avoidable via `Cnt` | **unavoidable** | avoidable via `Cnt` |
| Host latency in the motion loop | none | every waypoint | none |
| Sensor-reactive | no | per-waypoint only | with buffering |
| Setup effort | low | medium | medium to high |
| Reteaching cost | high | none | none for the dynamic case |
| Safety supervision | operator present | remote | remote |

Method B is **not** a superset of Method A. It gains programmatic control and
loses continuous motion. This non-obvious trade is, in the author's view, the
most useful thing to communicate about legacy robot integration.

---

## 10. Notes for publication

### 10.1 What is defensible as a contribution

- A documented, reproducible account of returning a specific obsolete
  manipulator and cell to service, including the failure modes encountered
- The **interface-granularity finding** of section 5, stated generally
- The **origin-recovery finding** of section 3.4, which contradicts commonly
  given advice
- The **negative result** on conveyor control, which is exhaustive rather than
  assumed, see the main README section 2.7
- A methodology for safely reverse-engineering an undocumented robot interface
- The **implemented teach-and-repeat system** of section 8: that a serial-only
  legacy controller can still be driven as a parameterised (per-point speed and
  I/O), **sensor-triggered** cell when reactivity is placed at the pass level
  rather than inside the motion loop — including the honest accounting that the
  per-point dwell is reduced (host-side redundancy removed) but not eliminated
  (physical deceleration remains; no `Cnt` over serial)
- The **tool-mounted-sensor gating pattern** of section 8.3: trusting a sensor
  that rides with the arm only at a known pose (a joint-tolerance at-home test),
  with a rising-edge trigger that needs no explicit part queue — a transferable
  idea for cells where the sensor cannot be fixed to the world frame
- The **safety architecture**: servo power strictly human-issued, a deterministic
  STOP that also halts an in-progress move and disarms automation, and the use
  of **automated tests** to pin motion-safety
  invariants (verified-syntax-only moves, frozen-joint handling, I/O-poll
  backoff) rather than relying on review alone

### 10.2 What is not yet defensible

| Claim | What is missing |
|---|---|
| Any precise cycle-time improvement figure | The per-point dead-time numbers in section 8.2 (~1.4 s → ~0.4 s) are coarse operator observations, not instrumented measurements with a timing harness and repeats |
| The meaning of the `CNT` arguments | Not confirmed from an authoritative source |
| Path deviation under `Cnt 1` | Never measured |
| Buffer depth required for Method C | Method C not yet implemented |

### 10.3 Experiments that would strengthen a paper

1. **Cycle time per method.** A fixed path of N waypoints, executed under
   Methods A, B and C, with host-side timestamps. Report mean and spread over
   repeated runs.
2. **Path deviation versus `Cnt` arguments.** Sweep the arguments, measure the
   deviation at the corner. This simultaneously establishes what the arguments
   mean, resolving section 7.1.
3. **Serial latency characterisation.** Round-trip time distribution for a
   null command, separating link latency from controller processing.
4. **Buffer depth versus segment duration** for Method C, establishing the
   lookahead needed to maintain continuous motion.
5. **Auto-run trigger reliability.** Over many parts, the false-trigger rate
   (a pass started with no part, or a mid-air reading acted on) and the
   missed-trigger rate, as a function of `AUTO_HOME_TOL_DEG`. This turns the
   at-home gating of section 8.3 from a plausible design choice into a measured
   one.
6. **Per-point dead-time, instrumented.** The ≈1.4 s→≈0.4 s claim of section 8.2
   repeated with host-side timestamps and repeats, separating the mechanical
   deceleration from the host-side overhead component.

### 10.4 Reproducibility

Everything needed to repeat the serial work is in section 4: link parameters,
frame format and the confirmed command set. The joint limits in section 1.2 are
cited to the manufacturer's manual. The failure modes in section 3.4 and the
negative result on conveyor control are reproducible on any comparable cell.

The implemented system of section 8 is reproducible from the repository: the
move sequence is section 4.3.1, the speed/settle/gripper parameters and the home
and `auto_start_point` values are in a single configuration file, and the
safety-critical invariants are pinned by the named automated tests cited in
sections 4.3.1, 8.2 and 10.1.

What is **not** reproducible from this document is the program-upload command
set of section 6.4, because it has not been established. A paper should either
establish it or scope around it explicitly.

---

## References

- [Mitsubishi Industrial Robot RV-1A / RV-2AJ Series Standard Specifications Manual, BFP-A8050](https://dl.mitsubishielectric.com/dl/fa/document/manual/robot/bfp-a8050/bfp-a8050k.pdf) — source of all specifications in sections 1.1 and 1.2
- [Mitsubishi CR1 / CR2 / CR3 / CR4 / CR7 / CR8 / CR9 Controller Instruction Manual, BFP-A5992](https://www.roboex.com/Mitsubishi%20Movemaster%20Robot%20Manual/RV-1A/CR1%20Controller%20Instruction%20Manual.pdf)
- [MELFA-BASIC IV command list, Mitsubishi RV-3SJ Series Standard Specifications Manual](https://www.manualslib.com/manual/1949566/Mitsubishi-Rv-3sj-Series.html?page=98) — source of the `CNT`, `FINE`, `ACCEL`, `OVRD`, `SPD` syntax
- [Mitsubishi MELFA CR1D Instruction Manual](https://www.manualslib.com/manual/2162775/Mitsubishi-Melfa-Cr1d.html) — contains the sections on Continuous Movement and target-position confirmation
