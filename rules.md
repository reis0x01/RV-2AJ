# rules.md — standing rules for the RV-2AJ project

Taken from the project's safety conventions. Add new rules as the user gives them.

## Safety (hardware)

- **Never command motion, servo, outputs or alarm resets on the real robot without the user's explicit go-ahead in that session.** Prefer `./start_robot.sh sim` for anything that does not need hardware.
- **Servo ON is only ever pressed by a person.** Code (auto-run, PLC link) may switch the servo off, never on.
- **OUT0–OUT3 are never touched** (factory-dedicated: running, servo on, error, operation rights). Outputs change only on explicit request; nothing moves or changes on connect.
- Speed: default 10 %, limit probe 5 %.
- Every planned move goes through the measured joint limits and MoveIt (self-collision, table, gripper). Station parts above the table are **not** modelled — do not assume they are.
- A joint already outside its range may stay or move back in, never further out.
- After an alarm or a USB drop, look at the robot before moving again. After an alarm reset the servo must be cycled off/on (driver does this).
- The controller's own Cartesian/circular moves (`EXECMOV PTMP`, `MVC`) use the uncorrected frame — keep them refused while zero offsets are non-zero.
- Hardware probing (robot I/O, the PLC) is read-only unless the user says otherwise. First contact with the real PLC: read DB10 only.
- The laptop writes only DB10 bytes 4–5; stoppers move only through the PLC (maintenance release pulses), never by writing PLC outputs directly. STOP ALL is a software stop, not a safety function: the hardware E-stop stays the safety stop.
- On the station PC (digital twin work over SSH): look and read first; install or change nothing without the user's OK.

## Working conventions

- Robot-specific numbers live only in `rv2aj_description/config/joint_limits.yaml`; home/auto-run settings in `rv2aj_control/config/rv2aj.yaml`. After changing them: rebuild + restart.
- Tests must keep the generic-robot fixture (`test/conftest.py`) so real calibration does not leak into tests.
- Only claim something is verified on hardware when it was; keep verified and unverified clearly apart in the docs.
- Back up taught points into `points_backups/<timestamp>/` before changes that could affect them.
- Keep everything offline-capable (no online-only services).
- Before deleting project files, make a full backup into `~/Documents/RV-2AJ-backups/` (user, 2026-10-27).
- README style (user, 2026-10-28): no dates, no history, no decision notes ("operator's choice"); plain technical prose that reads as written by a person.
