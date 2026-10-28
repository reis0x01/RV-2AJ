"""Operator GUI for the RV-2AJ stack. A pure ROS client: it never opens the
serial port, it calls the driver/sequencer/voice/translator interfaces.

Human confirmations live here (servo on, first J3 step, go-to point, loop
start, voice on); the hard safety rules are enforced by the driver itself.
"""

import json
import math
import sys
import threading
import time
from datetime import datetime

import rclpy
from PyQt5.QtCore import QEvent, QObject, Qt, QTimer, pyqtSignal
from PyQt5.QtGui import QColor, QKeySequence
from PyQt5.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QFrame,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QListWidget,
    QListWidgetItem,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QScrollArea,
    QShortcut,
    QSlider,
    QSpinBox,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)
from rcl_interfaces.msg import Log
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger

from rv2aj_interfaces.action import JointMove, TcpMove
from rv2aj_interfaces.msg import MotionCommand, RobotStatus, SequencerStatus
from rv2aj_interfaces.srv import (
    Connect,
    LineSignal,
    PointName,
    RunProgram,
    SavePoint,
    SetOverride,
    SetAutoRun,
    SetPointIO,
    SetPointSpeed,
)

from . import kinematics, serial_link
from .voice_commands import DIRECTIONS
from .widgets import THEME_QSS, Card, GlassBackground, JointCard, Segmented, StatCell, StatusPill

DEFAULT_OVERRIDE_PERCENT = 10
STEP_SIZES_DEG = (1.0, 5.0, 10.0, 20.0, 45.0)
TCP_STEPS_MM = (5.0, 10.0, 20.0, 50.0)
JOINT_NAMES = ("J1", "J2", "J3", "J4", "J5", "J6")
MOVABLE_JOINTS = ("J1", "J2", "J3", "J5", "J6")
MISCALIBRATED_JOINTS = {"J3"}
STATUS_TIMEOUT_S = 3.0
LOG_LIMIT = 300
SIDE_COLUMN_WIDTH = 320   # Gripper + Robot IO column; the IO card's own content needs ~310px
# A point's own speed override (see sequencer_node._transition_override): the "Slow"/"Fast"
# buttons set a SELECTED point to one of these two values. Edit these two numbers to
# change what "Slow"/"Fast" mean; the point's own exact speed stays editable per-point
# regardless (operator's choice, 2026-10-20).
SLOW_OVERRIDE_PERCENT = 35
FAST_OVERRIDE_PERCENT = 60
LOG_COLORS = {"info": "#3a3a3c", "ok": "#1f9d45", "warn": "#c46a00", "error": "#d70015"}
ROS_LEVEL = {10: "info", 20: "info", 30: "warn", 40: "error", 50: "error"}
TCP_BUTTONS = (
    ("↑  Up", "up"), ("←  Left", "left"), ("Forward", "forward"),
    ("↓  Down", "down"), ("Right  →", "right"), ("Back", "back"),
)


class RosBridge(QObject):
    """rclpy node spinning on its own thread; results come back as Qt signals."""

    status = pyqtSignal(object)
    sequencer = pyqtSignal(object)
    voice_text = pyqtSignal(str)
    voice_command = pyqtSignal(object)
    voice_status = pyqtSignal(str)
    log = pyqtSignal(str, str)
    done = pyqtSignal(str, bool, str)   # what, success, message
    plc_state = pyqtSignal(str)          # JSON from plc_bridge

    def __init__(self):
        super().__init__()
        self.node = Node("gui")
        n = self.node
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        n.create_subscription(RobotStatus, "status", lambda m: self.status.emit(m), 10)
        n.create_subscription(SequencerStatus, "sequencer/status", lambda m: self.sequencer.emit(m), latched)
        n.create_subscription(String, "voice/text", lambda m: self.voice_text.emit(m.data), 10)
        n.create_subscription(MotionCommand, "voice/command", lambda m: self.voice_command.emit(m), 10)
        n.create_subscription(String, "voice/status", lambda m: self.voice_status.emit(m.data), latched)
        n.create_subscription(Log, "/rosout", self._on_rosout, 50)
        n.create_subscription(String, "plc/state", lambda m: self.plc_state.emit(m.data), latched)

        self.clients = {
            "connect": n.create_client(Connect, "connect"),
            "disconnect": n.create_client(Trigger, "disconnect"),
            "servo": n.create_client(SetBool, "servo"),
            "stop": n.create_client(Trigger, "stop"),
            "reset_alarm": n.create_client(Trigger, "reset_alarm"),
            "set_override": n.create_client(SetOverride, "set_override"),
            "run_program": n.create_client(RunProgram, "run_program"),
            "points/save": n.create_client(SavePoint, "points/save"),
            "points/set_speed": n.create_client(SetPointSpeed, "points/set_speed"),
            "points/set_io": n.create_client(SetPointIO, "points/set_io"),
            "points/delete": n.create_client(PointName, "points/delete"),
            "points/clear": n.create_client(Trigger, "points/clear"),
            "points/goto": n.create_client(PointName, "points/goto"),
            "loop/start": n.create_client(Trigger, "loop/start"),
            "loop/stop": n.create_client(Trigger, "loop/stop"),
            "loop/auto": n.create_client(SetAutoRun, "loop/auto"),
            "voice/enable": n.create_client(SetBool, "voice/enable"),
            "gripper/open": n.create_client(Trigger, "gripper/open"),
            "home/goto": n.create_client(Trigger, "home/goto"),
            "gripper/close": n.create_client(Trigger, "gripper/close"),
            "line/set": n.create_client(LineSignal, "line/set"),
            "plc/maintenance": n.create_client(SetBool, "plc/maintenance"),
            "plc/release_1": n.create_client(Trigger, "plc/release_1"),
            "plc/release_2": n.create_client(Trigger, "plc/release_2"),
            "plc/release_3": n.create_client(Trigger, "plc/release_3"),
            "plc/line_stop": n.create_client(Trigger, "plc/line_stop"),
            "plc/line_reset": n.create_client(Trigger, "plc/line_reset"),
        }
        planned = n.declare_parameter("use_planner", True).value
        prefix = "planned/" if planned else ""
        self.tcp_action = ActionClient(n, TcpMove, prefix + "tcp_move")
        self.joint_action = ActionClient(n, JointMove, prefix + "joint_move")
        self.planned = planned
        self.translator_params = AsyncParameterClient(n, "translator")
        self.namespace = n.get_namespace().strip("/")

        self.executor = MultiThreadedExecutor(num_threads=2)
        self.executor.add_node(n)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()

    def _on_rosout(self, msg):
        if msg.name.split(".")[0] != self.namespace or msg.name.endswith(".gui"):
            return
        self.log.emit(ROS_LEVEL.get(msg.level, "info"), f"[{msg.name.split('.')[-1]}] {msg.msg}")

    DISCOVERY_WAIT_S = 3.0

    def call(self, name, request, what=None):
        """Call a service without blocking the UI; waits briefly for discovery."""
        what = what or name
        client = self.clients[name]

        def run():
            if not client.wait_for_service(timeout_sec=self.DISCOVERY_WAIT_S):
                self.done.emit(what, False, f"service {name} is not available (is the stack running?)")
                return

            def finished(f):
                response = f.result()
                if response is None:
                    self.done.emit(what, False, "no response")
                else:
                    self.done.emit(what, response.success, response.message)

            client.call_async(request).add_done_callback(finished)

        threading.Thread(target=run, daemon=True).start()

    def send_goal(self, client, goal, what):
        def run():
            if not client.wait_for_server(timeout_sec=self.DISCOVERY_WAIT_S):
                self.done.emit(what, False, f"{client._action_name} is not available (is the stack running?)")
                return

            def accepted(f):
                handle = f.result()
                if handle is None or not handle.accepted:
                    self.done.emit(what, False, "goal rejected (another move is running?)")
                    return
                handle.get_result_async().add_done_callback(
                    lambda r: self.done.emit(what, r.result().result.success, r.result().result.message)
                )

            client.send_goal_async(goal).add_done_callback(accepted)

        threading.Thread(target=run, daemon=True).start()

    def set_translator_param(self, name, value):
        if self.translator_params.services_are_ready():
            self.translator_params.set_parameters([Parameter(name, value=value)])

    def shutdown(self):
        self.executor.shutdown()
        self.node.destroy_node()



class _HotKeys(QObject):
    """Application-wide key, seen before any widget gets it:
    "+" (main or numpad)  emergency stop, always, even while typing in a field."""

    def __init__(self, window):
        super().__init__(window)
        self.window = window

    def eventFilter(self, obj, event):
        if (event.type() == QEvent.KeyPress and not event.isAutoRepeat()
                and event.key() == Qt.Key_Plus):
            self.window._on_stop()
            return True
        return False

class MainWindow(QMainWindow):
    """The operator window. One ROS 2 state (self.robot: RobotStatus, self.seq:
    SequencerStatus) drives every widget; there is no other source of truth here.

    Layout, top to bottom:
      header (never scrolls: connection/servo/alarm pills, STOP)
      two tabs, each inside a QScrollArea so nothing ever overlaps if a card needs
      more room than the window currently has:
        Operation (_build_body_scroll_area):
          left column:  Joints, then a row of  TCP | [Gripper, Robot IO]
          middle column: Motion (servo/speed/step/home), Points & loop, Event log
        Maintenance & safety (_build_maintenance_scroll_area): STOP ALL, robot
          home/reset/gripper, conveyor stoppers (PLC), every robot I/O signal

    Method groups, in the order they appear below:
      _build_*      construct widgets once, in __init__
      _render_line / _on_status / _on_sequencer / _refresh   ROS state -> widgets
      _on_*         user-initiated actions (button clicks, hotkeys, voice)
    Every action that can move the robot asks for a one-time confirmation per
    session (self._j3_confirmed etc.) before its first use; the driver enforces
    the actual safety rules independently of what this GUI does.
    """

    def __init__(self, bridge):
        super().__init__()
        self.ros = bridge
        self.setWindowTitle("RV-2AJ Controller · ROS 2")
        # The real size is set at the end of __init__, once the body (and so its
        # natural size) exists.

        self.robot = None
        self.last_status_time = 0.0
        self.seq = None
        self.plc = None   # last plc/state from plc_bridge (dict), None if never heard
        self.pending = set()        # GUI-initiated actions/services still running
        self.voice_state = "off"
        self._srvon_confirmed = False
        self._j3_confirmed = False
        self._goto_confirmed = False
        self._voice_confirmed = False
        self._maint_confirmed = False
        # STOP ALL latches here until "Reset stop": Servo ON, loop and auto-run stay
        # disabled meanwhile (the PLC latches its own line stop independently).
        self.estop_latched = False

        root = GlassBackground()
        self._background = root  # the scroll area re-invalidates its painted shadows against this
        self.setCentralWidget(root)
        outer = QVBoxLayout(root)
        outer.setContentsMargins(20, 16, 20, 20)
        outer.setSpacing(16)
        # The header holds STOP and the connection/servo/alarm state: it must never
        # scroll out of view, so it sits outside the scroll area below.
        outer.addWidget(self._build_header())
        self.tabs = QTabWidget()
        self.tabs.setObjectName("pages")
        self.tabs.setDocumentMode(True)
        self.tabs.addTab(self._build_body_scroll_area(), "Operation")
        self.tabs.addTab(self._build_maintenance_scroll_area(), "Maintenance && safety")
        self.tabs.currentChanged.connect(lambda _i: self._background.invalidate())
        outer.addWidget(self.tabs, 1)

        # Open maximized (fills the screen, keeps the window controls); the body
        # scroll area ensures everything stays reachable at any size.
        self.setMinimumSize(900, 620)
        self.resize(1280, 920)   # fallback size if un-maximized

        QShortcut(QKeySequence(Qt.Key_Escape), self, activated=self._on_stop,
                  context=Qt.ApplicationShortcut)
        # F11 toggles full-screen (the window opens full-screen; this gets the title
        # bar back and lets the operator move/resize or close the window if needed).
        QShortcut(QKeySequence(Qt.Key_F11), self, activated=self._toggle_fullscreen,
                  context=Qt.ApplicationShortcut)
        # "+" = emergency stop everywhere (even while typing).
        self._keys = _HotKeys(self)
        QApplication.instance().installEventFilter(self._keys)

        bridge.status.connect(self._on_status)
        bridge.sequencer.connect(self._on_sequencer)
        bridge.voice_text.connect(self._on_voice_text)
        bridge.voice_command.connect(self._on_voice_command)
        bridge.voice_status.connect(self._on_voice_status)
        bridge.log.connect(self._log)
        bridge.done.connect(self._on_done)
        bridge.plc_state.connect(self._on_plc_state)

        self.watchdog = QTimer(self)
        self.watchdog.timeout.connect(self._refresh)
        self.watchdog.start(1000)
        self._refresh()
        self._log("info", "GUI started. Waiting for the driver…")

    # -- layout ---------------------------------------------------------------

    def _build_header(self):
        header = QWidget()
        layout = QHBoxLayout(header)
        layout.setContentsMargins(4, 0, 0, 0)
        layout.setSpacing(12)
        titles = QVBoxLayout()
        titles.setSpacing(0)
        title = QLabel("RV-2AJ Controller")
        title.setObjectName("appTitle")
        subtitle = QLabel("Mitsubishi CR1-571  ·  ROS 2 · "
                          + ("MoveIt-validated motion" if self.ros.planned else "direct motion (no MoveIt)"))
        subtitle.setObjectName("appSubtitle")
        titles.addWidget(title)
        titles.addWidget(subtitle)
        layout.addLayout(titles)
        layout.addSpacing(16)

        self.link_pill = StatusPill()
        self.servo_pill = StatusPill()
        self.alarm_pill = StatusPill()
        for pill in (self.link_pill, self.servo_pill, self.alarm_pill):
            layout.addWidget(pill)
        layout.addStretch(1)

        self.port_combo = QComboBox()
        self.port_combo.setEditable(True)
        self.port_combo.setMinimumWidth(170)
        ports = serial_link.list_available_ports()
        if serial_link.PORT_DEFAULT not in ports:
            ports.insert(0, serial_link.PORT_DEFAULT)
        self.port_combo.addItems(ports)
        self.port_combo.setCurrentText(serial_link.PORT_DEFAULT)
        self.connect_btn = QPushButton("Connect")
        self.connect_btn.setObjectName("primary")
        self.connect_btn.setMinimumWidth(110)
        self.connect_btn.clicked.connect(self._on_connect_toggled)
        layout.addWidget(self.port_combo)
        layout.addWidget(self.connect_btn)
        layout.addSpacing(12)

        self.stop_btn = QPushButton("STOP")
        self.stop_btn.setObjectName("estop")
        self.stop_btn.setToolTip("Stop motion and any loop  ( + or Esc )")
        self.stop_btn.setCursor(Qt.PointingHandCursor)
        self.stop_btn.clicked.connect(self._on_stop)
        layout.addWidget(self.stop_btn)
        return header

    def _build_body_scroll_area(self):
        """Everything below the header: Joints/TCP on the left, Motion/Points/Log in
        the middle. Wrapped in a QScrollArea so that on a
        small screen, or if a card ever needs more room than it's given, the content
        SCROLLS instead of being squeezed into overlapping widgets."""
        left = QVBoxLayout()
        left.setSpacing(16)
        left.addWidget(self._build_joints_card())
        bottom = QHBoxLayout()
        bottom.setSpacing(16)
        bottom.addWidget(self._build_tcp_card(), 1)
        side = QVBoxLayout()
        side.setSpacing(16)
        side.setContentsMargins(0, 0, 0, 0)
        side.addWidget(self._build_gripper_card())
        side.addWidget(self._build_line_card())
        side.addStretch(1)
        side_widget = QWidget()
        side_widget.setLayout(side)
        side_widget.setFixedWidth(SIDE_COLUMN_WIDTH)
        bottom.addWidget(side_widget)
        left.addLayout(bottom)

        right_widget = QWidget()
        right_widget.setFixedWidth(380)
        right = QVBoxLayout(right_widget)
        right.setContentsMargins(0, 0, 0, 0)
        right.setSpacing(16)
        right.addWidget(self._build_motion_card())
        right.addWidget(self._build_points_card(), 3)
        right.addWidget(self._build_log_card(), 2)

        body = QHBoxLayout()
        body.setSpacing(16)
        body.addLayout(left, 1)
        body.addWidget(right_widget)

        return self._scroll_area(body)

    def _scroll_area(self, layout):
        """A page body: `layout` inside a transparent QScrollArea."""
        body_widget = QWidget()
        body_widget.setObjectName("bodyScrollContent")
        body_widget.setLayout(layout)

        scroll = QScrollArea()
        scroll.setObjectName("bodyScroll")
        scroll.setFrameShape(QFrame.NoFrame)
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        scroll.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        scroll.setWidget(body_widget)
        # GlassBackground caches each Card's painted drop-shadow at a fixed screen
        # position and only repaints it on that Card's own resize/move/show/hide
        # (see widgets.Card). Scrolling moves cards on screen without firing any of
        # those (their position relative to this scroll area's content widget never
        # changes), so without this the shadows would freeze in place while the
        # cards they belong to scroll away underneath them.
        scroll.horizontalScrollBar().valueChanged.connect(self._background.invalidate)
        scroll.verticalScrollBar().valueChanged.connect(self._background.invalidate)
        return scroll

    def _build_joints_card(self):
        card = Card("Joints")
        grid = QGridLayout()
        grid.setSpacing(12)
        self.joint_cards = {}
        for i, name in enumerate(JOINT_NAMES):
            jc = JointCard(name, absent=name not in MOVABLE_JOINTS,
                           warning="check origin" if name in MISCALIBRATED_JOINTS else None,
                           locked="LOCKED · encoder fault" if name in kinematics.FROZEN_JOINTS else None)
            jc.step_clicked.connect(lambda sign, n=name: self._on_joint_step(n, sign * self.step_size.value()))
            self.joint_cards[name] = jc
            grid.addWidget(jc, i // 3, i % 3)
        # No row stretch: each JointCard keeps its own natural (compact) height
        # instead of ballooning to fill whatever space a tall window happens to give it.
        card.body.addLayout(grid)
        return card

    def _build_tcp_card(self):
        card = Card("TCP")
        row = QHBoxLayout()
        row.setSpacing(24)
        self.tool_cells = {}
        for key, unit in (("X", "mm"), ("Y", "mm"), ("Z", "mm"), ("A", "°"), ("B", "°")):
            cell = StatCell(key, unit)
            self.tool_cells[key] = cell
            row.addWidget(cell)
        row.addStretch(1)
        card.body.addLayout(row)
        self.moving_label = QLabel()
        self.moving_label.setObjectName("jointHint")
        self.moving_label.setProperty("kind", "accent")
        card.body.addWidget(self.moving_label)
        card.body.addLayout(self._build_program_row())

        dist_row = QHBoxLayout()
        dist_label = QLabel("Distance per move (mm)")
        dist_label.setObjectName("dim")
        self.tcp_step = Segmented([(f"{s:g}", s) for s in TCP_STEPS_MM], default_index=1)
        self.tcp_step.setFixedWidth(220)
        self.tcp_step.changed.connect(lambda v: self.ros.set_translator_param("default_distance_mm", float(v)))
        dist_row.addWidget(dist_label)
        dist_row.addStretch(1)
        dist_row.addWidget(self.tcp_step)
        card.body.addLayout(dist_row)

        pad = QGridLayout()
        pad.setSpacing(8)
        self.tcp_buttons = []
        for i, (label, direction) in enumerate(TCP_BUTTONS):
            axis, sign = DIRECTIONS[direction]
            btn = QPushButton(label)
            btn.setObjectName("stepBtn")
            btn.setCursor(Qt.PointingHandCursor)
            btn.setToolTip("out along / back along the arm's reach" if axis == "R"
                           else f"{'+' if sign > 0 else '−'}{axis} (base frame)")
            btn.clicked.connect(lambda _=False, a=axis, s=sign, d=direction:
                                self._on_tcp_move(a, s * self.tcp_step.value(), d))
            self.tcp_buttons.append(btn)
            pad.addWidget(btn, i // 3, i % 3)
        card.body.addLayout(pad)

        voice_row = QHBoxLayout()
        voice_row.setSpacing(8)
        self.voice_btn = QPushButton("●  Voice off")
        self.voice_btn.setObjectName("voiceBtn")
        self.voice_btn.setCheckable(True)
        self.voice_btn.clicked.connect(self._on_voice_toggled)
        self.voice_state_label = QLabel("voice node: —")
        self.voice_state_label.setObjectName("dim")
        self.require_keyword = QCheckBox("Require “move”")
        self.require_keyword.setChecked(True)
        self.require_keyword.toggled.connect(lambda v: self.ros.set_translator_param("require_keyword", bool(v)))
        voice_row.addWidget(self.voice_btn)
        voice_row.addWidget(self.voice_state_label, 1)
        voice_row.addWidget(self.require_keyword)
        card.body.addLayout(voice_row)

        self.voice_heard = QLabel("Say “move up 20”, “go left 10”, “go to point 2”, “start loop”, “stop”.")
        self.voice_heard.setObjectName("dim")
        self.voice_heard.setWordWrap(True)
        card.body.addWidget(self.voice_heard)
        return card

    def _build_motion_card(self):
        card = Card("Motion")
        servo_row = QHBoxLayout()
        servo_row.setSpacing(8)
        self.servo_on_btn = QPushButton("Servo ON")
        self.servo_on_btn.setObjectName("servoOn")
        self.servo_on_btn.clicked.connect(self._on_servo_on)
        self.servo_off_btn = QPushButton("Servo OFF")
        self.servo_off_btn.setObjectName("servoOff")
        self.servo_off_btn.clicked.connect(lambda: self._call("servo", SetBool.Request(data=False), "servo off"))
        servo_row.addWidget(self.servo_on_btn)
        servo_row.addWidget(self.servo_off_btn)
        card.body.addLayout(servo_row)

        speed_head = QHBoxLayout()
        speed_label = QLabel("Speed override")
        speed_label.setObjectName("dim")
        self.speed_value = QLabel(f"{DEFAULT_OVERRIDE_PERCENT}%")
        self.speed_value.setObjectName("midValue")
        speed_head.addWidget(speed_label)
        speed_head.addStretch(1)
        speed_head.addWidget(self.speed_value)
        card.body.addLayout(speed_head)
        self.speed_slider = QSlider(Qt.Horizontal)
        self.speed_slider.setRange(5, 100)
        self.speed_slider.setValue(DEFAULT_OVERRIDE_PERCENT)
        self.speed_slider.valueChanged.connect(lambda v: self.speed_value.setText(f"{v}%"))
        self.speed_slider.sliderReleased.connect(
            lambda: self._call("set_override", SetOverride.Request(percent=self.speed_slider.value()),
                               "override"))
        card.body.addWidget(self.speed_slider)

        step_label = QLabel("Joint step size")
        step_label.setObjectName("dim")
        card.body.addWidget(step_label)
        self.step_size = Segmented([(f"{s:g}°", s) for s in STEP_SIZES_DEG], default_index=1)
        self.step_size.changed.connect(self._apply_step_size)
        card.body.addWidget(self.step_size)
        self._apply_step_size(self.step_size.value())

        self.reset_alarm_btn = QPushButton("Reset alarm")
        self.reset_alarm_btn.clicked.connect(lambda: self._call("reset_alarm", Trigger.Request()))
        buttons = QHBoxLayout()
        buttons.setSpacing(8)
        self.home_btn = QPushButton("⌂  Home")
        self.home_btn.setObjectName("homeBtn")
        self.home_btn.setToolTip("Joint move to the saved home pose (home_joints_deg)")
        self.home_btn.clicked.connect(self._on_home)
        buttons.addWidget(self.home_btn)
        buttons.addWidget(self.reset_alarm_btn)
        card.body.addLayout(buttons)
        return card

    def _build_gripper_card(self):
        card = Card("Gripper")
        row = QHBoxLayout()
        row.setSpacing(8)
        # Plain clicks: open/close are discrete state commands, not motion.
        self.gripper_open_btn = QPushButton("OPEN")
        self.gripper_open_btn.clicked.connect(lambda: self._on_gripper(True))
        self.gripper_close_btn = QPushButton("CLOSE")
        self.gripper_close_btn.clicked.connect(lambda: self._on_gripper(False))
        row.addWidget(self.gripper_open_btn)
        row.addWidget(self.gripper_close_btn)
        card.body.addLayout(row)
        self.gripper_state = QLabel("Last commanded: Unknown")
        self.gripper_state.setObjectName("dim")
        self.gripper_state.setWordWrap(True)
        card.body.addWidget(self.gripper_state)
        warning = QLabel("⚠ 160 N at 6 bar: keep fingers clear.")
        warning.setObjectName("note")
        warning.setWordWrap(True)
        card.body.addWidget(warning)
        self.gripper_error = ""
        return card

    # Robot I/O in use on Station 4 (survey 2026-10-16): general 0-15, hand 900-907. All outputs writable.
    # (row title, status field, first bit number, is output)
    IO_ROWS = (("IN 0", "line_inputs", 0, False), ("IN 8", "inputs_8_15", 8, False),
               ("OUT 0", "line_outputs", 0, True), ("OUT 8", "outputs_8_15", 8, True),
               ("EL G", "hand_inputs", 900, False), ("EL Ç", "hand_outputs", 900, True))
    IO_NAMES = {("IN", 1): "IN1 · TCP parça sensörü",
                ("OUT", 0): "OUT0 · çalışıyor (fabrika)", ("OUT", 1): "OUT1 · servo açık (fabrika)",
                ("OUT", 2): "OUT2 · hata (fabrika)", ("OUT", 3): "OUT3 · işletme yetkisi (fabrika)",
                ("IN", 6): "IN6 · sipariş bit 0", ("IN", 7): "IN7 · sipariş bit 1",
                ("OUT", 4): "OUT4 · hata kodu bit 0", ("OUT", 5): "OUT5 · hata kodu bit 1",
                ("OUT", 6): "OUT6 · Meşgul", ("OUT", 7): "OUT7 · İstasyon hazır",
                ("OUT", 900): "OUT900 · el 1 (tutucu)", ("OUT", 901): "OUT901 · el 1 (tutucu)"}

    def _add_io_row(self, grid, row, title, field, first, is_out, store=None):
        """One row of the Robot IO grid: a row label plus 8 LED buttons for bits
        `first`..`first+7` of one status byte (see IO_ROWS). Output rows are
        clickable (toggle that one bit); input rows are read-only indicators.
        Stores the row's buttons in store[field] (default self.io_leds) for
        _render_line to update; with a store given, every row is read-only."""
        clickable = is_out and store is None
        store = self.io_leds if store is None else store
        label = QLabel(title)
        label.setObjectName("ioRowLabel")
        grid.addWidget(label, row, 0)
        leds = []
        for b in range(8):
            n = first + b
            kind = "OUT" if is_out else "IN"
            led = QPushButton(str(n % 10) if first < 900 else str(b))
            led.setObjectName("ioLed")
            led.setFocusPolicy(Qt.NoFocus)
            led.setToolTip(self.IO_NAMES.get((kind, n), f"{kind}{n}") +
                           ("  (tıkla: aç/kapat)" if clickable else ""))
            led.setProperty("clickable", clickable)
            if clickable:
                led.clicked.connect(lambda _=False, f=field, n=n, b=b: self._on_out_click(f, n, b))
            else:
                led.setAttribute(Qt.WA_TransparentForMouseEvents, True)
            grid.addWidget(led, row, b + 1)
            leds.append(led)
        store[field] = leds

    def _build_line_card(self):
        card = Card("Robot IO · İstasyon 4")
        row = QHBoxLayout()
        self.line_order = QLabel("—")
        self.line_order.setObjectName("orderValue")
        self.line_bits = QLabel("")
        self.line_bits.setObjectName("dim")
        row.addWidget(self.line_order)
        row.addStretch(1)
        row.addWidget(self.line_bits)
        card.body.addLayout(row)
        grid = QGridLayout()
        grid.setHorizontalSpacing(3)
        grid.setVerticalSpacing(5)
        self.io_leds = {}   # status field -> [8 LEDs], filled in by _add_io_row below
        for r, (title, field, first, is_out) in enumerate(self.IO_ROWS):
            self._add_io_row(grid, r, title, field, first, is_out)
        card.body.addLayout(grid)
        legend = QLabel("Çıkışlar tıklanabilir · EL = el (pnömatik) IO 900-907 · 900/901 tutucu")
        legend.setObjectName("dim")
        legend.setWordWrap(True)
        card.body.addWidget(legend)
        self.io_record_box = QCheckBox("Tıklananı seçili noktaya kaydet")
        self.io_record_box.setToolTip("Açıkken, bir çıkışa tıklamak onu robotta da değiştirir VE seçili "
                                      "noktaya kaydeder: nokta oraya ulaşınca bu çıkış otomatik ayarlanır "
                                      "(tutucu açma/kapama gibi).")
        card.body.addWidget(self.io_record_box)
        toggles = QHBoxLayout()
        self.line_ready_btn = QPushButton("Hazır")
        self.line_busy_btn = QPushButton("Meşgul")
        for btn, signal in ((self.line_ready_btn, "ready"), (self.line_busy_btn, "busy")):
            btn.setObjectName("ioToggle")
            btn.clicked.connect(lambda _=False, sig=signal: self._on_line_toggle(sig))
            toggles.addWidget(btn, 1)
        card.body.addLayout(toggles)
        error_row = QHBoxLayout()
        error_label = QLabel("Hata kodu")
        error_label.setObjectName("dim")
        self.line_error = Segmented([(str(c), c) for c in range(4)])
        self.line_error.setToolTip("İstasyona gönderilen hata kodu (0 = hata yok)")
        self.line_error.changed.connect(lambda code: self._call(
            "line/set", LineSignal.Request(signal="error_code", value=int(code)), f"hata kodu {code}"))
        error_row.addWidget(error_label)
        error_row.addWidget(self.line_error, 1)
        card.body.addLayout(error_row)
        self.line_reset_btn = QPushButton("Genel çıkışları sıfırla (OUT0-15)")
        self.line_reset_btn.setToolTip("El (pnömatik) çıkışlarına dokunmaz: valfler aniden yön değiştirmesin.")
        self.line_reset_btn.clicked.connect(self._on_line_reset)
        card.body.addWidget(self.line_reset_btn)
        return card

    def _on_out_click(self, field, n, b):
        """Toggle one output; the driver writes that bit only and reads its byte back.
        With "record to point" checked and a point selected, also save that output +
        value against the point (set after it's reached, same slot as the gripper)."""
        r = self.robot
        value = getattr(r, field, -1) if r is not None else -1
        if value < 0:
            return
        on = not (value >> b) & 1
        self._call("line/set", LineSignal.Request(signal=f"out{n}", value=int(on)),
                   f"OUT{n} {'açık' if on else 'kapalı'}")
        name = self._selected_point()
        if self.io_record_box.isChecked() and name:
            self._call("points/set_io", SetPointIO.Request(name=name, output=n, value=bool(on)),
                       f"{name}: OUT{n} = {int(on)} kaydedildi")

    def _on_line_reset(self):
        r = self.robot
        if r is None:
            return
        for field, first in (("line_outputs", 0), ("outputs_8_15", 8)):
            value = getattr(r, field, -1)
            for b in range(8):
                if value >= 0 and (value >> b) & 1:
                    self._call("line/set", LineSignal.Request(signal=f"out{first + b}", value=0),
                               f"OUT{first + b} kapalı")

    def _on_line_toggle(self, signal):
        r = self.robot
        if r is None or r.line_outputs < 0:
            return
        bit = 7 if signal == "ready" else 6
        on = not (r.line_outputs >> bit) & 1
        self._call("line/set", LineSignal.Request(signal=signal, value=int(on)),
                   f"{'hazır' if signal == 'ready' else 'meşgul'} {'açık' if on else 'kapalı'}")

    @staticmethod
    def _set_led(led, on):
        if led.property("on") != on:
            led.setProperty("on", on)
            led.style().unpolish(led)
            led.style().polish(led)

    def _render_line(self, r):
        if r is None or r.line_inputs < 0 or r.line_outputs < 0 or not r.connected:
            self.line_order.setText("—")
            self.line_bits.setText("IO okunamıyor" if r is not None and r.connected else "")
            for w in (self.line_ready_btn, self.line_busy_btn, self.line_error, self.line_reset_btn):
                w.setEnabled(False)
            for store in (self.io_leds, self.maint_io_leds):
                for leds in store.values():
                    for led in leds:
                        self._set_led(led, False)
                        led.setEnabled(False)
            for label in self.named_io_labels.values():
                label.setText(label.property("name") + "   —")
            return
        for title, field, first, is_out in self.IO_ROWS:
            value = getattr(r, field, -1)
            for store in (self.io_leds, self.maint_io_leds):
                for b, led in enumerate(store[field]):
                    self._set_led(led, value >= 0 and bool((value >> b) & 1))
                    led.setEnabled(value >= 0)
        for (kind, n), label in self.named_io_labels.items():
            value = getattr(r, self._io_field(kind, n), -1)
            bit = n - 900 if n >= 900 else n % 8
            state = "—" if value < 0 else ("● ON" if (value >> bit) & 1 else "○ off")
            label.setText(f"{label.property('name')}   {state}")
        order = {(0, 0): "Passif", (0, 1): "Montaj", (1, 0): "Ayıklama", (1, 1): "Paletten al"}[
            ((r.line_inputs >> 7) & 1, (r.line_inputs >> 6) & 1)]
        self.line_order.setText(order)
        self.line_order.setProperty("kind", "" if order == "Passif" else "active")
        self.line_order.style().unpolish(self.line_order)
        self.line_order.style().polish(self.line_order)
        self.line_bits.setText(f"IN {r.line_inputs:08b}  OUT {r.line_outputs:08b}")
        for btn, bit, name in ((self.line_ready_btn, 7, "Hazır"), (self.line_busy_btn, 6, "Meşgul")):
            on = bool((r.line_outputs >> bit) & 1)
            btn.setText(f"{name}  {'AÇIK' if on else 'KAPALI'}")
            btn.setProperty("active", on)
            btn.style().unpolish(btn)
            btn.style().polish(btn)
            btn.setEnabled(True)
        self.line_error.set_value(((r.line_outputs >> 5) & 1) << 1 | ((r.line_outputs >> 4) & 1))
        self.line_error.setEnabled(True)
        self.line_reset_btn.setEnabled(True)

    @staticmethod
    def _io_field(kind, n):
        """RobotStatus field holding signal IN/OUT n."""
        if n >= 900:
            return "hand_inputs" if kind == "IN" else "hand_outputs"
        if n >= 8:
            return "inputs_8_15" if kind == "IN" else "outputs_8_15"
        return "line_inputs" if kind == "IN" else "line_outputs"

    # -- maintenance & safety page ----------------------------------------------------

    STATION_NAMES = ("1 · robot", "2 · filling", "3 · full cars")

    def _build_maintenance_scroll_area(self):
        left_widget = QWidget()
        left_widget.setFixedWidth(380)
        left = QVBoxLayout(left_widget)
        left.setContentsMargins(0, 0, 0, 0)
        left.setSpacing(16)
        left.addWidget(self._build_safety_card())
        left.addWidget(self._build_service_robot_card())
        left.addStretch(1)
        middle = QVBoxLayout()
        middle.setSpacing(16)
        middle.addWidget(self._build_conveyor_card())
        middle.addStretch(1)
        right = QVBoxLayout()
        right.setSpacing(16)
        right.addWidget(self._build_all_io_card())
        right.addStretch(1)
        body = QHBoxLayout()
        body.setSpacing(16)
        body.addWidget(left_widget)
        body.addLayout(middle, 1)
        body.addLayout(right, 1)
        return self._scroll_area(body)

    def _build_safety_card(self):
        card = Card("Emergency stop")
        self.stop_all_btn = QPushButton("STOP ALL")
        self.stop_all_btn.setObjectName("estop")
        self.stop_all_btn.setMinimumHeight(64)
        self.stop_all_btn.setCursor(Qt.PointingHandCursor)
        self.stop_all_btn.setToolTip("Robot and conveyor; stays stopped until Reset stop")
        self.stop_all_btn.clicked.connect(self._on_stop_all)
        card.body.addWidget(self.stop_all_btn)
        what = QLabel("Robot: STOP, loop and auto-run off, servo off (the gripper stays as it is, "
                      "so a held part is not dropped).\nConveyor: every stopper up, all cars held "
                      "(latched in the PLC).")
        what.setObjectName("dim")
        what.setWordWrap(True)
        card.body.addWidget(what)
        note = QLabel("⚠ Software stop over USB and the network. The red hardware E-stop is the "
                      "safety stop.")
        note.setObjectName("note")
        note.setWordWrap(True)
        card.body.addWidget(note)
        self.estop_state = QLabel()
        self.estop_state.setWordWrap(True)
        card.body.addWidget(self.estop_state)
        self.estop_reset_btn = QPushButton("Reset stop")
        self.estop_reset_btn.clicked.connect(self._on_estop_reset)
        card.body.addWidget(self.estop_reset_btn)
        return card

    def _build_service_robot_card(self):
        card = Card("Robot")
        self.maint_robot_state = QLabel()
        self.maint_robot_state.setObjectName("dim")
        self.maint_robot_state.setWordWrap(True)
        card.body.addWidget(self.maint_robot_state)
        servo_row = QHBoxLayout()
        servo_row.setSpacing(8)
        self.maint_servo_on_btn = QPushButton("Servo ON")
        self.maint_servo_on_btn.setObjectName("servoOn")
        self.maint_servo_on_btn.clicked.connect(self._on_servo_on)
        self.maint_servo_off_btn = QPushButton("Servo OFF")
        self.maint_servo_off_btn.setObjectName("servoOff")
        self.maint_servo_off_btn.clicked.connect(
            lambda: self._call("servo", SetBool.Request(data=False), "servo off"))
        servo_row.addWidget(self.maint_servo_on_btn)
        servo_row.addWidget(self.maint_servo_off_btn)
        card.body.addLayout(servo_row)
        row = QHBoxLayout()
        row.setSpacing(8)
        self.maint_home_btn = QPushButton("⌂  Home")
        self.maint_home_btn.setObjectName("homeBtn")
        self.maint_home_btn.setToolTip("Joint move to the saved home pose (needs Servo ON)")
        self.maint_home_btn.clicked.connect(self._on_home)
        self.maint_reset_alarm_btn = QPushButton("Reset alarm")
        self.maint_reset_alarm_btn.clicked.connect(lambda: self._call("reset_alarm", Trigger.Request()))
        row.addWidget(self.maint_home_btn)
        row.addWidget(self.maint_reset_alarm_btn)
        card.body.addLayout(row)
        grip = QHBoxLayout()
        grip.setSpacing(8)
        self.maint_gripper_open_btn = QPushButton("Gripper OPEN")
        self.maint_gripper_open_btn.clicked.connect(lambda: self._on_gripper(True))
        self.maint_gripper_close_btn = QPushButton("Gripper CLOSE")
        self.maint_gripper_close_btn.clicked.connect(lambda: self._on_gripper(False))
        grip.addWidget(self.maint_gripper_open_btn)
        grip.addWidget(self.maint_gripper_close_btn)
        card.body.addLayout(grip)
        warning = QLabel("⚠ 160 N at 6 bar: keep fingers clear.")
        warning.setObjectName("note")
        card.body.addWidget(warning)
        return card

    def _build_conveyor_card(self):
        card = Card("Conveyor stoppers · PLC")
        self.plc_link_label = QLabel()
        self.plc_link_label.setObjectName("dim")
        self.plc_link_label.setWordWrap(True)
        card.body.addWidget(self.plc_link_label)
        self.maint_btn = QPushButton("Maintenance mode  OFF")
        self.maint_btn.setObjectName("ioToggle")
        self.maint_btn.setToolTip("Pauses the PLC's automatic decisions; stoppers then open only "
                                  "with Release. Not while auto-run is armed.")
        self.maint_btn.clicked.connect(self._on_maintenance_toggle)
        card.body.addWidget(self.maint_btn)
        grid = QGridLayout()
        grid.setHorizontalSpacing(12)
        grid.setVerticalSpacing(8)
        for col, title in enumerate(("Station", "Car", "Parts", "Piston", "")):
            label = QLabel(title)
            label.setObjectName("ioRowLabel")
            grid.addWidget(label, 0, col)
        self.station_widgets = []
        for i, name in enumerate(self.STATION_NAMES):
            grid.addWidget(QLabel(name), i + 1, 0)
            leds = []
            for col in (1, 2):
                led = QPushButton("")
                led.setObjectName("ioLed")
                led.setFocusPolicy(Qt.NoFocus)
                led.setAttribute(Qt.WA_TransparentForMouseEvents, True)
                grid.addWidget(led, i + 1, col)
                leds.append(led)
            piston = QLabel("—")
            piston.setObjectName("dim")
            grid.addWidget(piston, i + 1, 3)
            release = QPushButton("Release")
            release.setToolTip(f"Lower stopper {i + 1} for one car (maintenance mode)")
            release.clicked.connect(lambda _=False, n=i + 1: self._on_release(n))
            grid.addWidget(release, i + 1, 4)
            self.station_widgets.append((leds[0], leds[1], piston, release))
        grid.setColumnStretch(3, 1)
        card.body.addLayout(grid)
        note = QLabel("Release lowers the piston for one car; it comes back up 1 s after the car has "
                      "gone. Station 1 not while the robot is busy. Cars on the belt keep moving "
                      "until a stopper holds them.")
        note.setObjectName("dim")
        note.setWordWrap(True)
        card.body.addWidget(note)
        return card

    def _build_all_io_card(self):
        card = Card("Robot I/O · all signals")
        grid = QGridLayout()
        grid.setHorizontalSpacing(3)
        grid.setVerticalSpacing(5)
        self.maint_io_leds = {}
        for r, (title, field, first, is_out) in enumerate(self.IO_ROWS):
            self._add_io_row(grid, r, title, field, first, is_out, store=self.maint_io_leds)
        card.body.addLayout(grid)
        legend = QLabel("Read-only here (outputs are switched on the Operation tab). "
                        "Hover a LED for its name.")
        legend.setObjectName("dim")
        legend.setWordWrap(True)
        card.body.addWidget(legend)
        self.named_io_labels = {}
        for kind, n in sorted(self.IO_NAMES, key=lambda k: (k[0] != "IN", k[1])):
            label = QLabel()
            label.setObjectName("namedIo")
            label.setProperty("name", self.IO_NAMES[(kind, n)])
            label.setText(self.IO_NAMES[(kind, n)] + "   —")
            card.body.addWidget(label)
            self.named_io_labels[(kind, n)] = label
        return card

    def _refresh_maintenance(self, connected, servo, moving, looping, ready):
        r = self.robot
        alarm = bool(connected and r.alarm)
        self.maint_robot_state.setText(
            ("Connected" if connected else "Not connected") + (f" · servo {'ON' if servo else 'off'}"
            f" · {'ALARM' if alarm else 'no alarm'} · {'moving' if moving else 'still'}" if connected else ""))
        self.maint_servo_on_btn.setEnabled(connected and not self.estop_latched)
        self.maint_servo_off_btn.setEnabled(connected)
        self.maint_reset_alarm_btn.setEnabled(connected)
        self.maint_home_btn.setEnabled(ready and "home" not in self.pending)
        gripper_ready = connected and not (self.pending & {"gripper open", "gripper close"})
        self.maint_gripper_open_btn.setEnabled(gripper_ready)
        self.maint_gripper_close_btn.setEnabled(gripper_ready)

        p = self.plc or {}
        plc_ok = bool(p.get("enabled") and p.get("connected") and p.get("plc_alive"))
        stopped = bool(p.get("line_stopped"))
        maint_on = bool(p.get("maint_active"))
        maint_req = bool(p.get("maint_requested"))
        auto = bool(self.seq and self.seq.auto_mode)
        robot_busy = looping or moving or bool(self.seq and self.seq.current_point)
        link = self._plc_text()
        if stopped:
            link += " · LINE STOPPED"
        elif maint_on:
            link += " · maintenance mode"
        self.plc_link_label.setText(link)
        self.maint_btn.setEnabled(plc_ok and not stopped and (maint_req or (not auto and not robot_busy)))
        self.maint_btn.setText("Maintenance mode  ON" if maint_on else
                               "Maintenance mode  (waiting for PLC)" if maint_req else "Maintenance mode  OFF")
        if self.maint_btn.property("active") != maint_on:
            self.maint_btn.setProperty("active", maint_on)
            self.maint_btn.style().unpolish(self.maint_btn)
            self.maint_btn.style().polish(self.maint_btn)
        stations = p.get("stations") or [{}, {}, {}]
        for i, (car, parts, piston, release) in enumerate(self.station_widgets):
            s = stations[i] if plc_ok and i < len(stations) else {}
            self._set_led(car, bool(s.get("car")))
            self._set_led(parts, bool(s.get("parts")))
            piston.setText("—" if not plc_ok else "DOWN · passing" if s.get("valve") else "UP · holding")
            release.setEnabled(plc_ok and maint_on and not stopped and not (i == 0 and robot_busy))

        lines = []
        if self.estop_latched:
            lines.append("STOP ALL active: robot servo off, auto-run off.")
        if stopped:
            lines.append("Conveyor: line stop latched in the PLC, all cars held.")
        elif p.get("line_stop_requested"):
            lines.append("Conveyor: stop requested but not confirmed by the PLC (not reachable?).")
        self.estop_state.setText("\n".join(lines) or "Not stopped.")
        self.estop_state.setObjectName("error" if lines else "dim")
        self.estop_state.style().unpolish(self.estop_state)
        self.estop_state.style().polish(self.estop_state)
        self.estop_reset_btn.setEnabled(bool(lines))

    def _on_stop_all(self):
        """Software emergency stop: robot motion, loop/auto-run, servo, and the conveyor line."""
        self.estop_latched = True
        self.ros.call("stop", Trigger.Request(), "stop")
        self.ros.call("loop/stop", Trigger.Request(), "end loop")
        self.ros.call("servo", SetBool.Request(data=False), "servo off")
        self.ros.call("plc/line_stop", Trigger.Request(), "line stop")
        self._log("error", "STOP ALL pressed: robot stopped, servo off, auto-run off, conveyor line stop")
        self._refresh()

    def _on_estop_reset(self):
        if not self._confirm("Reset stop",
                             "Check that the robot area and the conveyor are clear.\n\n"
                             "The conveyor's automatic logic resumes (cars may move). The robot stays "
                             "servo off: press Servo ON yourself, then re-arm auto-run if wanted."):
            return
        self.estop_latched = False
        p = self.plc or {}
        if p.get("line_stopped") or p.get("line_stop_requested"):
            self._call("plc/line_reset", Trigger.Request(), "line reset")
        self._log("ok", "Stop reset")
        self._refresh()

    def _on_maintenance_toggle(self):
        want = not (self.plc or {}).get("maint_requested")
        if want and not self._maint_confirmed:
            if not self._confirm("Maintenance mode",
                                 "The PLC stops its automatic decisions: no robot passes are requested "
                                 "and no stopper opens by itself. A stopper opens only when you press "
                                 "Release.\n\nCars already released keep moving. Keep hands clear of "
                                 "the conveyor."):
                return
            self._maint_confirmed = True
        self._call("plc/maintenance", SetBool.Request(data=want),
                   "maintenance on" if want else "maintenance off")

    def _on_release(self, station):
        self._log("info", f"Release station {station}")
        self._call(f"plc/release_{station}", Trigger.Request(), f"release station {station}")

    def _build_program_row(self):
        """Controller program: one compact row inside the TCP card (RUN is rarely used)."""
        row = QHBoxLayout()
        row.setSpacing(8)
        self.program_spin = QSpinBox()
        self.program_spin.setRange(1, 9999)
        self.program_spin.setPrefix("Prog  ")
        self.program_spin.setButtonSymbols(QSpinBox.NoButtons)
        self.program_spin.setFixedWidth(110)
        self.run_btn = QPushButton("Run")
        self.run_btn.setToolTip("RUN has not been tested on this controller yet.")
        self.run_btn.clicked.connect(
            lambda: self._call("run_program", RunProgram.Request(program=self.program_spin.value()), "run"))
        self.program_stop_btn = QPushButton("Stop")
        self.program_stop_btn.clicked.connect(self._on_stop)
        self.state_raw = QLineEdit()
        self.state_raw.setObjectName("stateRaw")
        self.state_raw.setReadOnly(True)
        self.state_raw.setPlaceholderText("STATE reply")
        row.addWidget(self.program_spin)
        row.addWidget(self.run_btn)
        row.addWidget(self.program_stop_btn)
        row.addWidget(self.state_raw, 1)
        return row

    def _build_points_card(self):
        card = Card("Points & loop")
        # Two rows instead of one cramped row of 5: every label gets enough room
        # to never truncate or overflow, whatever font the system substitutes.
        edit_row1 = QHBoxLayout()
        edit_row1.setSpacing(8)
        self.save_point_btn = QPushButton("＋ Save point")
        self.save_point_btn.setObjectName("primary")
        self.save_point_btn.clicked.connect(lambda: self._call("points/save", SavePoint.Request(), "save point"))
        self.insert_point_btn = QPushButton("＋ Insert after")
        self.insert_point_btn.setToolTip("Save the current pose as a new point right after the selected one "
                                         "(e.g. between P1 and P2: select P1)")
        self.insert_point_btn.clicked.connect(self._on_insert_point)
        edit_row1.addWidget(self.save_point_btn, 1)
        edit_row1.addWidget(self.insert_point_btn, 1)
        card.body.addLayout(edit_row1)

        edit_row2 = QHBoxLayout()
        edit_row2.setSpacing(8)
        self.goto_btn = QPushButton("Go to")
        self.goto_btn.clicked.connect(self._on_goto)
        self.delete_point_btn = QPushButton("Delete")
        self.delete_point_btn.clicked.connect(self._on_delete_point)
        self.clear_points_btn = QPushButton("Clear")
        self.clear_points_btn.clicked.connect(self._on_clear_points)
        edit_row2.addWidget(self.goto_btn, 1)
        edit_row2.addWidget(self.delete_point_btn, 1)
        edit_row2.addWidget(self.clear_points_btn, 1)
        card.body.addLayout(edit_row2)

        speed_label = QLabel("Speed of the move TO the selected point (from the point before it):")
        speed_label.setObjectName("dim")
        speed_label.setWordWrap(True)
        card.body.addWidget(speed_label)
        speed_row = QHBoxLayout()
        speed_row.setSpacing(8)
        self.slow_point_btn = QPushButton(f"Slow ({SLOW_OVERRIDE_PERCENT}%)")
        self.slow_point_btn.setToolTip("The robot travels slowly from the previous point to the selected one. "
                                       "To leave it slowly as well, also mark the next point Slow.")
        self.slow_point_btn.clicked.connect(lambda: self._on_set_point_speed(SLOW_OVERRIDE_PERCENT))
        self.fast_point_btn = QPushButton(f"Fast ({FAST_OVERRIDE_PERCENT}%)")
        self.fast_point_btn.setToolTip("The robot travels fast from the previous point to the selected one.")
        self.fast_point_btn.clicked.connect(lambda: self._on_set_point_speed(FAST_OVERRIDE_PERCENT))
        speed_row.addWidget(self.slow_point_btn, 1)
        speed_row.addWidget(self.fast_point_btn, 1)
        card.body.addLayout(speed_row)

        io_row = QHBoxLayout()
        io_row.setSpacing(8)
        self.point_io_label = QLabel("")
        self.point_io_label.setObjectName("dim")
        self.point_io_label.setWordWrap(True)
        self.clear_point_io_btn = QPushButton("Çıkışları temizle")
        self.clear_point_io_btn.setToolTip("Bu noktaya kaydedilmiş tüm çıkış eylemlerini kaldırır "
                                          "(noktanın kendisini veya tutucusunu etkilemez).")
        self.clear_point_io_btn.clicked.connect(self._on_clear_point_io)
        io_row.addWidget(self.point_io_label, 1)
        io_row.addWidget(self.clear_point_io_btn)
        card.body.addLayout(io_row)

        header = QLabel("      " + "".join(f"{j:>7}" for j in MOVABLE_JOINTS) + "   Grip  Spd")
        header.setObjectName("pointsHeader")
        card.body.addWidget(header)
        self.points_list = QListWidget()
        self.points_list.setObjectName("points")
        self.points_list.setMinimumHeight(110)
        self.points_list.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.points_list.itemSelectionChanged.connect(self._refresh)
        self.points_list.itemDoubleClicked.connect(lambda _item: self._on_goto())
        card.body.addWidget(self.points_list, 1)

        loop_row = QHBoxLayout()
        loop_row.setSpacing(8)
        self.loop_start_btn = QPushButton("▶  Start loop")
        self.loop_start_btn.setObjectName("loopStart")
        self.loop_start_btn.clicked.connect(self._on_loop_start)
        self.loop_end_btn = QPushButton("■  End loop")
        self.loop_end_btn.setObjectName("loopEnd")
        self.loop_end_btn.clicked.connect(lambda: self._call("loop/stop", Trigger.Request(), "end loop"))
        loop_row.addWidget(self.loop_start_btn, 1)
        loop_row.addWidget(self.loop_end_btn, 1)
        card.body.addLayout(loop_row)
        # Sensor-triggered auto-run: a TCP part sensor runs one pass per part, only
        # when the arm is parked at home (readings during a pass are ignored).
        self.auto_run_btn = QPushButton(self._auto_run_label())
        self.auto_run_btn.setObjectName("loopStart")
        self.auto_run_btn.setCheckable(True)
        self.auto_run_btn.setToolTip("Açıkken: robot EVDE beklerken parça hazır sinyali gelince (TCP sensörü "
                                     "ya da konveyör PLC'si) tüm noktaları bir kez çalıştırır, sonra eve döner "
                                     "ve sıradaki parçayı bekler. Servo açık olmalı. "
                                     "STOP / + oto-çalışmayı kapatır.")
        self.auto_run_btn.clicked.connect(self._on_auto_run)
        card.body.addWidget(self.auto_run_btn)
        self.loop_status = QLabel()
        self.loop_status.setObjectName("dim")
        self.loop_status.setWordWrap(True)
        card.body.addWidget(self.loop_status)
        return card

    def _build_log_card(self):
        card = Card("Event log")
        self.log_list = QListWidget()
        self.log_list.setObjectName("log")
        self.log_list.setWordWrap(True)
        card.body.addWidget(self.log_list, 1)
        return card

    def _log(self, kind, text):
        item = QListWidgetItem(f"{datetime.now():%H:%M:%S}  {text}")
        item.setForeground(QColor(LOG_COLORS.get(kind, LOG_COLORS["info"])))
        self.log_list.addItem(item)
        while self.log_list.count() > LOG_LIMIT:
            self.log_list.takeItem(0)
        self.log_list.scrollToBottom()

    def _confirm(self, title, text):
        return QMessageBox.warning(self, title, text, QMessageBox.Yes | QMessageBox.No,
                                   QMessageBox.No) == QMessageBox.Yes

    def _call(self, service, request, what=None):
        what = what or service
        self.pending.add(what)
        self._refresh()
        self.ros.call(service, request, what)

    def _apply_step_size(self, size):
        for jc in self.joint_cards.values():
            jc.set_step_size(size)

    def _driver_alive(self):
        return self.robot is not None and time.monotonic() - self.last_status_time < STATUS_TIMEOUT_S

    def _selected_point(self):
        item = self.points_list.currentItem()
        return item.data(Qt.UserRole) if item is not None and item.isSelected() else None

    def _selected_teachpoint(self):
        """The full TeachPoint message for the selected point (speed, gripper, IO
        actions), or None if nothing is selected or the sequencer isn't connected yet."""
        name = self._selected_point()
        if name is None or self.seq is None:
            return None
        return next((p for p in self.seq.points if p.name == name), None)

    # -- state -> UI ----------------------------------------------------------------

    def _refresh(self):
        alive = self._driver_alive()
        r = self.robot if alive else None
        connected = bool(r and r.connected)
        servo = bool(connected and r.servo_on)
        moving = bool(connected and r.moving)
        looping = bool(self.seq and self.seq.loop_active)
        motion_pending = bool(self.pending & {"tcp move", "joint step", "servo on"})

        if not alive:
            self.link_pill.set_state("● NO DRIVER", "error" if self.robot else "idle")
        elif connected:
            self.link_pill.set_state("● CONNECTED", "ok")
        elif r.last_error:
            self.link_pill.set_state("● LINK ERROR", "error")
        else:
            self.link_pill.set_state("● OFFLINE", "idle")
        if not connected:
            self.servo_pill.set_state("SERVO —", "idle")
        else:
            self.servo_pill.set_state("SERVO ON" if servo else "SERVO OFF", "warn" if servo else "idle")
        if connected and r.alarm:
            self.alarm_pill.set_state("ALARM", "error")
        elif self.estop_latched or (self.plc or {}).get("line_stopped"):
            self.alarm_pill.set_state("STOPPED · reset on Maintenance", "error")
        elif looping:
            self.alarm_pill.set_state(f"LOOP · CYCLE {self.seq.cycle}", "info")
        else:
            self.alarm_pill.set_state("NO ALARM", "idle")

        for w in (self.servo_off_btn, self.reset_alarm_btn, self.speed_slider):
            w.setEnabled(connected)
        self.servo_on_btn.setEnabled(connected and not self.estop_latched)
        for w in (self.run_btn, self.program_spin):
            w.setEnabled(connected and not looping and not moving)
        self.program_stop_btn.setEnabled(connected)
        # Pneumatic: needs a connection, not servo power.
        gripper_ready = connected and not (self.pending & {"gripper open", "gripper close"})
        self.gripper_open_btn.setEnabled(gripper_ready)
        self.gripper_close_btn.setEnabled(gripper_ready)
        state = {"open": "Open", "closed": "Closed"}.get(r.gripper if connected else "", "Unknown")
        if self.gripper_error:
            self.gripper_state.setText(f"Last commanded: {state}\nRejected: {self.gripper_error}")
            self.gripper_state.setObjectName("error")
        else:
            self.gripper_state.setText(f"Last commanded: {state}")
            self.gripper_state.setObjectName("dim")
        self.gripper_state.style().unpolish(self.gripper_state)
        self.gripper_state.style().polish(self.gripper_state)

        ready = servo and not moving and not looping and not motion_pending
        for jc in self.joint_cards.values():
            jc.set_steps_enabled(ready)
        for btn in self.tcp_buttons:
            btn.setEnabled(ready)

        have_points = bool(self.seq and self.seq.points)
        selected = self._selected_point() is not None
        self.save_point_btn.setEnabled(connected and not moving and not looping)
        self.insert_point_btn.setEnabled(connected and not moving and not looping and selected)
        self.slow_point_btn.setEnabled(selected and not looping)
        self.fast_point_btn.setEnabled(selected and not looping)
        self.io_record_box.setEnabled(selected)
        if not selected and self.io_record_box.isChecked():
            self.io_record_box.setChecked(False)
        point = self._selected_teachpoint()
        has_io = bool(point and point.io_outputs)
        self.clear_point_io_btn.setEnabled(has_io and not looping)
        if point is None:
            self.point_io_label.setText("")
        elif not point.io_outputs:
            self.point_io_label.setText(f"{point.name}: kayıtlı çıkış yok")
        else:
            actions = ", ".join(f"OUT{o}={int(v)}" for o, v in zip(point.io_outputs, point.io_values))
            self.point_io_label.setText(f"{point.name} vardığında: {actions}")
        self.goto_btn.setEnabled(ready and selected)
        self.home_btn.setEnabled(ready and "home" not in self.pending)
        self.delete_point_btn.setEnabled(selected and not looping)
        self.clear_points_btn.setEnabled(have_points and not looping)
        auto = bool(self.seq and self.seq.auto_mode)
        pending = self.seq.pending_parts if self.seq else 0
        auto_bit = self.seq.auto_input_bit if self.seq else 5
        self.loop_start_btn.setEnabled(ready and self.seq is not None and len(self.seq.points) >= 2 and not auto
                                       and not self.estop_latched)
        self.loop_end_btn.setEnabled(looping and not auto)
        # Auto-run can only be armed when the servo is on and there is at least one point
        # (and not during STOP ALL or conveyor maintenance).
        maint = bool((self.plc or {}).get("maint_requested"))
        self.auto_run_btn.setEnabled((auto or (servo and not looping and not self.estop_latched and not maint))
                                     and bool(have_points))
        if self.auto_run_btn.isChecked() != auto:
            self.auto_run_btn.setChecked(auto)
        self.auto_run_btn.setText("■  Oto-çalışmayı durdur" if auto else self._auto_run_label())
        plc_mode = bool(self.seq and self.seq.auto_trigger == "plc")
        source = "PLC" if plc_mode else f"IN{auto_bit}"
        plc_text = f" · {self._plc_text()}" if plc_mode else ""
        if auto:
            if looping:
                run = f"parça işleniyor → {self.seq.current_point or '…'}"
            else:
                run = "evde parça bekleniyor"
            q = f" · sırada {pending} parça" if pending else ""
            self.loop_status.setText(f"Oto-çalışma AÇIK ({source}) · {run}{q}{plc_text}")
        elif looping:
            self.loop_status.setText(f"Looping · cycle {self.seq.cycle} · moving to {self.seq.current_point or '…'}")
        elif self.seq is None:
            self.loop_status.setText("Sequencer not running.")
        elif len(self.seq.points) < 2:
            self.loop_status.setText("Save at least 2 points to loop.")
        else:
            self.loop_status.setText(f"{len(self.seq.points)} points ready.{plc_text}")

        self._refresh_maintenance(connected, servo, moving, looping, ready)

        self.connect_btn.setEnabled(alive)
        self.port_combo.setEnabled(alive and not connected)
        self.connect_btn.setText("Disconnect" if connected else "Connect")
        self.connect_btn.setObjectName("" if connected else "primary")
        self.connect_btn.style().unpolish(self.connect_btn)
        self.connect_btn.style().polish(self.connect_btn)
        self.voice_btn.setEnabled(True)

    def _on_status(self, msg):
        first = self.robot is None
        was_connected = bool(self.robot and self.robot.connected)
        self.robot = msg
        self.last_status_time = time.monotonic()
        self._render_line(msg)
        if first:
            self._log("ok", "Driver found")
        if was_connected and not msg.connected and msg.last_error:
            self._log("error", f"Link dropped: {msg.last_error}")
        for i, name in enumerate(JOINT_NAMES):
            value = msg.joints_deg[i]
            self.joint_cards[name].set_value(None if math.isnan(value) else value)
        for key, value in (("X", msg.x_mm), ("Y", msg.y_mm), ("Z", msg.z_mm), ("A", msg.a_deg), ("B", msg.b_deg)):
            self.tool_cells[key].set_value(None if math.isnan(value) else value)
        self.state_raw.setText(msg.state_raw)
        self.moving_label.setText(f"moving: {msg.active_motion}" if msg.moving else "")
        if not self.speed_slider.isSliderDown() and msg.connected:
            self.speed_slider.setValue(msg.override_percent)
        self._refresh()

    def _on_insert_point(self):
        name = self._selected_point()
        if name:
            self._call("points/save", SavePoint.Request(after=name), f"insert after {name}")

    def _on_set_point_speed(self, percent):
        name = self._selected_point()
        if name:
            self._call("points/set_speed", SetPointSpeed.Request(name=name, percent=percent),
                       f"{name} speed {percent}%")

    def _on_clear_point_io(self):
        point = self._selected_teachpoint()
        if point is None:
            return
        for output in point.io_outputs:
            self._call("points/set_io", SetPointIO.Request(name=point.name, output=int(output), clear=True),
                       f"{point.name}: OUT{output} kaldırıldı")

    def _on_sequencer(self, msg):
        selected = self._selected_point()
        self.seq = msg
        self.points_list.clear()
        for p in msg.points:
            angles = "".join(f"{p.joints_deg[i]:>7.1f}" for i, j in enumerate(JOINT_NAMES) if j in MOVABLE_JOINTS)
            grip = {"open": "open", "closed": "close"}.get(p.gripper, "  -")
            item = QListWidgetItem(f"{p.name:<4}{angles}   {grip} {p.speed_percent:4d}%")
            item.setData(Qt.UserRole, p.name)
            self.points_list.addItem(item)
            if p.name == selected or (msg.loop_active and p.name == msg.current_point):
                item.setSelected(True)
                self.points_list.setCurrentItem(item)
        self._refresh()

    def _on_done(self, what, success, message):
        self.pending.discard(what)
        if what in ("gripper open", "gripper close"):
            self.gripper_error = "" if success else message
        if not success:
            self._log("warn" if not what.startswith("gripper") else "error", f"{what}: {message}")
        self._refresh()

    def _on_gripper(self, open_):
        self.gripper_error = ""
        self._call("gripper/open" if open_ else "gripper/close", Trigger.Request(),
                   "gripper open" if open_ else "gripper close")

    # -- actions ------------------------------------------------------------------

    def _on_connect_toggled(self):
        if self.robot and self.robot.connected:
            self._call("disconnect", Trigger.Request())
        else:
            self._call("connect", Connect.Request(port=self.port_combo.currentText().strip()))

    def _on_servo_on(self):
        if not self._srvon_confirmed:
            if not self._confirm("Confirm servo on", "Robot will move when jogged. Clear the area."):
                return
            self._srvon_confirmed = True
        self._call("servo", SetBool.Request(data=True), "servo on")

    def _on_stop(self):
        self.ros.call("stop", Trigger.Request(), "stop")
        if self.seq and self.seq.loop_active:
            self.ros.call("loop/stop", Trigger.Request(), "end loop")
        self._log("error", "STOP pressed")

    def _on_joint_step(self, joint, offset):
        if joint in MISCALIBRATED_JOINTS and not self._j3_confirmed:
            if not self._confirm(f"Confirm {joint} move",
                                 f"{joint}'s origin is miscalibrated on this robot, so its angle reading "
                                 f"and soft limits may be wrong. Step {joint} anyway?"):
                return
            self._j3_confirmed = True
        goal = JointMove.Goal(relative=True, joint=joint, offset_deg=float(offset))
        self.pending.add("joint step")
        self._refresh()
        self.ros.send_goal(self.ros.joint_action, goal, "joint step")

    def _on_tcp_move(self, axis, distance, direction):
        goal = TcpMove.Goal(axis=axis, distance_mm=float(distance))
        self.pending.add("tcp move")
        self._refresh()
        self._log("info", f"TCP {direction} {abs(distance):g} mm [button]")
        self.ros.send_goal(self.ros.tcp_action, goal, "tcp move")

    def _on_home(self):
        if not self._goto_confirmed:
            if not self._confirm("Confirm move home",
                                 "The robot moves every joint straight to the home pose. "
                                 "This can be a large move. Clear the area."):
                return
            self._goto_confirmed = True
        self._call("home/goto", Trigger.Request(), "home")

    def _on_goto(self):
        name = self._selected_point()
        if name is None:
            return
        if not self._goto_confirmed:
            if not self._confirm("Confirm move to point",
                                 "The robot moves every joint straight to the saved angles. "
                                 "This can be a large move. Clear the area."):
                return
            self._goto_confirmed = True
        self._call("points/goto", PointName.Request(name=name), "go to")

    def _on_delete_point(self):
        name = self._selected_point()
        if name is not None:
            self._call("points/delete", PointName.Request(name=name), "delete point")

    def _on_clear_points(self):
        if self.seq and self._confirm("Clear points", f"Delete all {len(self.seq.points)} saved points?"):
            self._call("points/clear", Trigger.Request(), "clear points")

    def _on_loop_start(self):
        if not self.seq or len(self.seq.points) < 2:
            return
        names = " → ".join(p.name for p in self.seq.points)
        if not self._confirm(
            "Start loop",
            f"The robot will move continuously through {len(self.seq.points)} points ({names} → "
            f"{self.seq.points[0].name} …) at {self.speed_slider.value()}% override until you press "
            f"End loop.\n\nMoves between points can be large. Clear the area.",
        ):
            return
        self._goto_confirmed = True
        self._call("loop/start", Trigger.Request(), "start loop")

    def _auto_run_label(self):
        plc_mode = bool(self.seq and self.seq.auto_trigger == "plc")
        return "▶  Oto-çalışma (PLC)" if plc_mode else "▶  Oto-çalışma (TCP sensörü)"

    def _plc_text(self):
        p = self.plc
        if p is None:
            return "PLC köprüsü çalışmıyor"
        if not p.get("enabled"):
            return "PLC kapalı (plc_ip boş)"
        if not p.get("connected"):
            return "PLC bağlantısı yok"
        if not p.get("plc_alive"):
            return "PLC yanıt vermiyor (STOP?)"
        text = "PLC: araba hazır" if p.get("part_ready") else "PLC: bağlı"
        if p.get("pass_count"):
            text += f", bu arabada {p['pass_count']}. geçiş"
        if p.get("missed_part"):
            text += " · ⚠ son araba parçayla gitti"
        return text

    def _on_plc_state(self, data):
        try:
            self.plc = json.loads(data)
        except ValueError:
            return
        self._refresh()

    def _on_auto_run(self):
        want_on = self.auto_run_btn.isChecked()
        armed = bool(self.seq and self.seq.auto_mode)
        bit = self.seq.auto_input_bit if self.seq else 5
        plc_mode = bool(self.seq and self.seq.auto_trigger == "plc")
        if want_on and not armed:
            n = len(self.seq.points) if self.seq else 0
            trigger = ("the conveyor PLC reports a car with parts at station 1" if plc_mode
                       else f"a part reaches the IN{bit} sensor")
            if not self._confirm(
                "Arm auto-run",
                f"The robot will start MOVING ON ITS OWN each time {trigger} — "
                f"one full pass through all {n} saved points each time.\n\n"
                f"The servo must stay on. Press STOP (or +) to disarm. Clear the area.",
            ):
                self.auto_run_btn.setChecked(False)
                return
            self._goto_confirmed = True
            self._call("loop/auto", SetAutoRun.Request(enable=True, input_bit=-1), "arm auto-run")
        elif not want_on:
            self._call("loop/auto", SetAutoRun.Request(enable=False, input_bit=-1), "disarm auto-run")

    # -- voice --------------------------------------------------------------------

    def _on_voice_toggled(self):
        want_on = self.voice_btn.isChecked()
        if want_on and not self._voice_confirmed:
            if not self._confirm(
                "Enable voice control",
                "Spoken commands will move the robot while the servo is on.\n\n"
                "Saying “stop” stops it, but recognition takes 1–2 seconds. Use the red STOP "
                "button or Esc for anything urgent. Clear the area.",
            ):
                self.voice_btn.setChecked(False)
                return
            self._voice_confirmed = True
        self.voice_btn.setChecked(self.voice_state not in ("off", "error"))
        self.ros.call("voice/enable", SetBool.Request(data=want_on), "voice")

    def _on_voice_status(self, state):
        self.voice_state = state
        on = state not in ("off", "error")
        self.voice_btn.setChecked(on)
        self.voice_btn.setText("●  Voice on" if on else "●  Voice off")
        self.voice_btn.setProperty("active", on)
        self.voice_btn.style().unpolish(self.voice_btn)
        self.voice_btn.style().polish(self.voice_btn)
        self.voice_state_label.setText(f"voice node: {state}")

    def _on_voice_text(self, text):
        self.voice_heard.setText(f"Heard: “{text}”")

    def _on_voice_command(self, cmd):
        if cmd.kind == MotionCommand.KIND_NONE:
            meaning = f"ignored ({cmd.reason})"
        elif cmd.kind == MotionCommand.KIND_TCP:
            where = "along arm" if cmd.axis == "R" else cmd.axis
            meaning = f"TCP {where} {cmd.distance_mm:+g} mm"
        elif cmd.kind == MotionCommand.KIND_GOTO_POINT:
            meaning = f"go to {cmd.point}"
        else:
            meaning = cmd.kind.replace("_", " ")
        self.voice_heard.setText(f"Heard: “{cmd.text}”  →  {meaning}")

    def _toggle_fullscreen(self):
        if self.isFullScreen():
            self.showNormal()
        else:
            self.showFullScreen()

    def closeEvent(self, event):
        self.watchdog.stop()
        super().closeEvent(event)


def main(args=None):
    import faulthandler
    faulthandler.enable()  # a native crash prints where it happened instead of dying silently
    rclpy.init(args=args)
    QApplication.setAttribute(Qt.AA_EnableHighDpiScaling, True)
    QApplication.setAttribute(Qt.AA_UseHighDpiPixmaps, True)
    app = QApplication(sys.argv[:1])
    app.setStyle("Fusion")
    app.setStyleSheet(THEME_QSS)
    bridge = RosBridge()
    window = MainWindow(bridge)
    window.showFullScreen()   # always open full-screen (F11 toggles the title bar back)
    code = app.exec_()
    bridge.shutdown()
    if rclpy.ok():
        rclpy.shutdown()
    sys.exit(code)


if __name__ == "__main__":
    main()
