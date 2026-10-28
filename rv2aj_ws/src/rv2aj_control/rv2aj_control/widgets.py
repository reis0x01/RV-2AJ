"""Theme and reusable widgets. All styling lives in THEME_QSS (plus the
backdrop colours in GlassBackground)."""

from PyQt5.QtCore import QPointF, QRectF, Qt, pyqtSignal
from PyQt5.QtGui import QColor, QFont, QLinearGradient, QPainter, QPixmap, QRadialGradient
from PyQt5.QtWidgets import (
    QButtonGroup,
    QFrame,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QSizePolicy,
    QVBoxLayout,
    QWidget,
)

UI_FONTS = '"SF Pro Text", "SF Pro Display", "Inter", "Helvetica Neue", "Segoe UI", "Ubuntu", sans-serif'
MONO_FONTS = '"SF Mono", "JetBrains Mono", "Menlo", "Ubuntu Mono", "DejaVu Sans Mono", monospace'

ACCENT = "#007aff"
TEXT = "#1d1d1f"
TEXT_DIM = "#6e6e73"

THEME_QSS = f"""
* {{
    font-family: {UI_FONTS};
    font-size: 13px;
    color: {TEXT};
}}

QFrame#card {{
    background: rgba(255, 255, 255, 0.55);
    border: 1px solid rgba(255, 255, 255, 0.90);
    border-radius: 18px;
}}

QLabel#cardTitle {{ color: {TEXT_DIM}; font-size: 11px; font-weight: 700; }}
QLabel#appTitle {{ font-size: 18px; font-weight: 700; }}
QLabel#appSubtitle {{ color: {TEXT_DIM}; font-size: 12px; }}
QLabel#dim {{ color: {TEXT_DIM}; }}
QLabel#note {{ color: #c46a00; font-size: 11px; }}
QLabel#error {{ color: #d70015; font-size: 12px; }}

QLabel#bigValue {{ font-family: {MONO_FONTS}; font-size: 34px; font-weight: 600; }}
QLabel#midValue {{ font-family: {MONO_FONTS}; font-size: 18px; font-weight: 600; }}
QLabel#unit {{ color: {TEXT_DIM}; font-size: 12px; }}
QLabel#jointName {{ font-size: 13px; font-weight: 700; color: #3a3a3c; }}
QLabel#jointHint {{ font-size: 11px; color: {TEXT_DIM}; }}
QLabel#jointHint[kind="accent"] {{ color: {ACCENT}; font-weight: 600; }}
QLabel#jointHint[kind="warn"] {{ color: #c46a00; }}

QFrame#jointCard {{
    background: rgba(255, 255, 255, 0.62);
    border: 1px solid rgba(255, 255, 255, 0.95);
    border-radius: 14px;
}}
QFrame#jointCard[moving="true"] {{
    background: rgba(0, 122, 255, 0.10);
    border: 1px solid {ACCENT};
}}
QFrame#jointCard[absent="true"] {{
    background: rgba(255, 255, 255, 0.25);
    border: 1px dashed rgba(0, 0, 0, 0.12);
}}

QLabel#pill {{ padding: 5px 11px; border-radius: 11px; font-size: 11px; font-weight: 700; }}
QLabel#pill[kind="idle"]  {{ background: rgba(255, 255, 255, 0.60); color: {TEXT_DIM}; }}
QLabel#pill[kind="ok"]    {{ background: rgba(52, 199, 89, 0.18);  color: #1f9d45; }}
QLabel#pill[kind="warn"]  {{ background: rgba(255, 149, 0, 0.20);  color: #c46a00; }}
QLabel#pill[kind="error"] {{ background: rgba(255, 59, 48, 0.16);  color: #d70015; }}
QLabel#pill[kind="info"]  {{ background: rgba(0, 122, 255, 0.14);  color: #0062cc; }}

QPushButton {{
    background: rgba(255, 255, 255, 0.72);
    border: 1px solid rgba(0, 0, 0, 0.06);
    border-radius: 10px;
    padding: 8px 14px;
    font-weight: 600;
}}
QPushButton:hover {{ background: rgba(255, 255, 255, 0.95); }}
QPushButton:pressed {{ background: rgba(228, 234, 244, 0.95); }}
QPushButton:disabled {{
    color: rgba(29, 29, 31, 0.28);
    background: rgba(255, 255, 255, 0.35);
    border-color: rgba(0, 0, 0, 0.03);
}}

QPushButton#primary {{ background: {ACCENT}; border: none; color: white; }}
QPushButton#primary:hover {{ background: #1a88ff; }}
QPushButton#primary:pressed {{ background: #0062cc; }}

QPushButton#servoOn {{ background: rgba(52, 199, 89, 0.16); border-color: rgba(52, 199, 89, 0.40); color: #1f9d45; }}
QPushButton#servoOn:hover {{ background: rgba(52, 199, 89, 0.26); }}
QPushButton#servoOn:disabled, QPushButton#servoOff:disabled {{
    color: rgba(29, 29, 31, 0.28);
    background: rgba(255, 255, 255, 0.35);
    border-color: rgba(0, 0, 0, 0.03);
}}

QPushButton#stepBtn {{ font-size: 13px; min-height: 34px; padding: 0 8px; }}
QPushButton#stepBtn:hover {{ background: rgba(0, 122, 255, 0.12); border-color: rgba(0, 122, 255, 0.45); color: {ACCENT}; }}
QPushButton#stepBtn:pressed {{ background: rgba(0, 122, 255, 0.22); }}

QPushButton#iconBtn {{ padding: 6px 10px; font-size: 15px; }}

QPushButton#homeBtn {{ background: rgba(0, 122, 255, 0.12); border: 1px solid rgba(0, 122, 255, 0.45); color: {ACCENT}; }}
QPushButton#homeBtn:hover {{ background: rgba(0, 122, 255, 0.22); }}
QPushButton#homeBtn:disabled {{ background: rgba(0, 122, 255, 0.05); border-color: rgba(0, 122, 255, 0.2); color: rgba(0, 122, 255, 0.45); }}

QPushButton#voiceBtn {{ min-width: 120px; }}
QPushButton#voiceBtn[active="true"] {{ background: {ACCENT}; border: none; color: white; }}
QPushButton#voiceBtn[active="true"]:hover {{ background: #1a88ff; }}
QLabel#orderValue {{ font-size: 15px; font-weight: 700; color: {TEXT_DIM}; }}
QLabel#orderValue[kind="active"] {{ color: {ACCENT}; }}
QPushButton#ioToggle[active="true"] {{ background: rgba(52, 199, 89, 0.18); border-color: rgba(52, 199, 89, 0.50); color: #1f9d45; }}
QPushButton#ioLed {{ min-width: 24px; max-width: 24px; min-height: 24px; max-height: 24px; padding: 0;
    border-radius: 12px; font-size: 10px; font-weight: 700; background: rgba(120, 120, 128, 0.14);
    border: 1px solid rgba(120, 120, 128, 0.30); color: {TEXT_DIM}; }}
QPushButton#ioLed[on="true"] {{ background: #34c759; border-color: #2a9d48; color: white; }}
QPushButton#ioLed[locked="true"] {{ background: rgba(120, 120, 128, 0.06); border-style: dashed; color: rgba(120, 120, 128, 0.55); }}
QPushButton#ioLed[locked="true"][on="true"] {{ background: rgba(52, 199, 89, 0.45); color: white; }}
QPushButton#ioLed[clickable="true"]:hover {{ border: 2px solid {ACCENT}; }}
QLabel#ioRowLabel {{ font-size: 11px; font-weight: 700; color: {TEXT_DIM}; min-width: 38px; }}

QScrollArea#bodyScroll, QScrollArea#bodyScroll > QWidget > QWidget {{ background: transparent; border: none; }}
QWidget#bodyScrollContent {{ background: transparent; }}
QTabWidget#pages::pane {{ border: none; background: transparent; }}
QTabWidget#pages > QTabBar::tab {{
    background: rgba(255, 255, 255, 0.40);
    border: 1px solid rgba(0, 0, 0, 0.06);
    border-radius: 9px;
    padding: 7px 18px;
    min-width: 150px;
    margin: 0 6px 10px 0;
    font-weight: 600;
    color: {TEXT_DIM};
}}
QTabWidget#pages > QTabBar::tab:hover {{ background: rgba(255, 255, 255, 0.70); }}
QTabWidget#pages > QTabBar::tab:selected {{ background: rgba(255, 255, 255, 0.92); color: {TEXT}; }}
QLabel#namedIo {{ font-family: {MONO_FONTS}; font-size: 11px; color: #3a3a3c; }}
QPushButton#pttBtn {{ min-height: 44px; font-size: 14px; }}
QPushButton#pttBtn[active="true"] {{ background: #ff3b30; border: none; color: white; }}
QLabel#aiTranscript {{ color: #3a3a3c; }}
QLabel#aiResponse {{ font-weight: 600; }}
QLabel#aiResponse[kind="ok"] {{ color: #1f9d45; }}
QLabel#aiResponse[kind="warn"] {{ color: #c46a00; }}
QLabel#aiResponse[kind="error"] {{ color: #d70015; }}
QLabel#aiResponse[kind="info"] {{ color: #0062cc; }}
QListWidget#aiChat {{
    background: rgba(255, 255, 255, 0.70);
    border: 1px solid rgba(0, 0, 0, 0.06);
    border-radius: 12px;
    font-size: 13px;
    padding: 6px;
}}
QListWidget#aiChat::item {{ padding: 4px 2px; }}
QListWidget#aiPlan {{
    background: rgba(255, 255, 255, 0.55);
    border: 1px solid rgba(0, 0, 0, 0.06);
    border-radius: 10px;
    font-family: {MONO_FONTS};
    font-size: 11px;
    padding: 4px;
}}
QCheckBox {{ color: #3a3a3c; spacing: 6px; }}

QPushButton#estop {{
    background: #ff3b30;
    border: 2px solid rgba(255, 255, 255, 0.75);
    border-radius: 12px;
    color: white;
    font-size: 16px;
    font-weight: 800;
    padding: 10px 28px;
}}
QPushButton#estop:hover {{ background: #ff5147; }}
QPushButton#estop:pressed {{ background: #d70015; }}

QFrame#segmented {{
    background: rgba(118, 118, 128, 0.12);
    border: none;
    border-radius: 9px;
}}
QPushButton#segment {{
    background: transparent;
    border: none;
    border-radius: 7px;
    padding: 6px 0;
    color: #3a3a3c;
    font-weight: 500;
}}
QPushButton#segment:hover {{ color: {TEXT}; }}
QPushButton#segment:checked {{
    background: white;
    border: 1px solid rgba(0, 0, 0, 0.04);
    color: {TEXT};
    font-weight: 700;
}}

QComboBox, QSpinBox {{
    background: rgba(255, 255, 255, 0.75);
    border: 1px solid rgba(0, 0, 0, 0.08);
    border-radius: 9px;
    padding: 6px 10px;
    min-height: 20px;
}}
QComboBox:disabled, QSpinBox:disabled {{ color: rgba(29, 29, 31, 0.35); background: rgba(255, 255, 255, 0.4); }}
QComboBox::drop-down {{ border: none; width: 22px; }}
QComboBox QAbstractItemView {{
    background: white;
    border: 1px solid rgba(0, 0, 0, 0.10);
    selection-background-color: {ACCENT};
    selection-color: white;
    outline: none;
}}

QSlider::groove:horizontal {{ height: 4px; background: rgba(0, 0, 0, 0.10); border-radius: 2px; }}
QSlider::sub-page:horizontal {{ background: {ACCENT}; border-radius: 2px; }}
QSlider::sub-page:horizontal:disabled {{ background: rgba(0, 0, 0, 0.18); }}
QSlider::handle:horizontal {{
    background: white;
    border: 1px solid rgba(0, 0, 0, 0.15);
    width: 18px;
    height: 18px;
    margin: -8px 0;
    border-radius: 10px;
}}

QLabel#pointsHeader {{ font-family: {MONO_FONTS}; font-size: 11px; color: {TEXT_DIM}; padding-left: 5px; }}

QListWidget#points {{
    background: rgba(255, 255, 255, 0.55);
    border: 1px solid rgba(0, 0, 0, 0.06);
    border-radius: 10px;
    padding: 4px;
    font-family: {MONO_FONTS};
    font-size: 12px;
    outline: none;
}}
QListWidget#points::item {{ padding: 7px 8px; border-radius: 7px; color: #3a3a3c; }}
QListWidget#points::item:hover {{ background: rgba(0, 0, 0, 0.04); }}
QListWidget#points::item:selected {{ background: rgba(0, 122, 255, 0.14); color: {TEXT}; }}

QPushButton#loopStart {{ background: #34c759; border: none; color: white; }}
QPushButton#loopStart:hover {{ background: #45d168; }}
QPushButton#loopStart:pressed {{ background: #28a745; }}
QPushButton#loopEnd {{ background: rgba(255, 59, 48, 0.14); border: 1px solid rgba(255, 59, 48, 0.40); color: #d70015; }}
QPushButton#loopEnd:hover {{ background: rgba(255, 59, 48, 0.24); }}
QPushButton#loopStart:disabled, QPushButton#loopEnd:disabled, QPushButton#primary:disabled {{
    color: rgba(29, 29, 31, 0.28);
    background: rgba(255, 255, 255, 0.35);
    border: 1px solid rgba(0, 0, 0, 0.03);
}}

QListWidget#log {{
    background: transparent;
    border: none;
    font-family: {MONO_FONTS};
    font-size: 12px;
    outline: none;
}}
QListWidget#log::item {{ padding: 3px 2px; border: none; }}
QListWidget#log::item:selected {{ background: rgba(0, 0, 0, 0.04); }}

QLineEdit#stateRaw {{
    font-family: {MONO_FONTS};
    font-size: 11px;
    color: {TEXT_DIM};
    background: rgba(255, 255, 255, 0.50);
    border: 1px solid rgba(0, 0, 0, 0.06);
    border-radius: 9px;
    padding: 6px 8px;
}}

QScrollBar:vertical {{ background: transparent; width: 8px; margin: 0; }}
QScrollBar::handle:vertical {{ background: rgba(0, 0, 0, 0.18); border-radius: 4px; min-height: 24px; }}
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {{ height: 0; }}
QScrollBar:horizontal {{ background: transparent; height: 8px; margin: 0; }}
QScrollBar::handle:horizontal {{ background: rgba(0, 0, 0, 0.18); border-radius: 4px; min-width: 24px; }}
QScrollBar::add-line:horizontal, QScrollBar::sub-line:horizontal {{ width: 0; }}

QToolTip {{
    background: white;
    color: {TEXT};
    border: 1px solid rgba(0, 0, 0, 0.10);
    padding: 6px 8px;
}}

QMessageBox {{ background: #f5f5f7; }}
QMessageBox QPushButton {{ min-width: 80px; }}
"""

# Soft colour fields the translucent cards sit on, for the frosted-glass look.
BACKDROP_BLOBS = (
    # centre x, centre y (fractions of the window), radius (fraction), colour
    (0.10, 0.05, 0.55, "#a9cbff"),
    (0.92, 0.15, 0.50, "#dcc8ff"),
    (0.65, 1.05, 0.60, "#ffd6c4"),
    (0.02, 0.95, 0.45, "#bdeee0"),
)


def repolish(widget):
    widget.style().unpolish(widget)
    widget.style().polish(widget)


class GlassBackground(QWidget):
    """Window backdrop: a pale gradient with blurred colour blobs."""

    _cache = None

    def invalidate(self):
        self._cache = None
        self.update()

    def paintEvent(self, event):
        # The backdrop never changes, so it is rendered once per window size and
        # then only copied; redrawing the gradients on every status update was
        # the GUI's main CPU cost.
        if self._cache is None or self._cache.size() != self.size():
            self._cache = QPixmap(self.size())
            self._render(self._cache)
        QPainter(self).drawPixmap(event.rect(), self._cache, event.rect())

    def _render(self, device):
        painter = QPainter(device)
        painter.setRenderHint(QPainter.Antialiasing)
        rect = device.rect()
        w, h = rect.width(), rect.height()

        base = QLinearGradient(0, 0, w, h)
        base.setColorAt(0, QColor("#eef3fb"))
        base.setColorAt(1, QColor("#f8f4f9"))
        painter.fillRect(rect, base)

        for cx, cy, radius, colour in BACKDROP_BLOBS:
            grad = QRadialGradient(QPointF(cx * w, cy * h), radius * max(w, h))
            inner = QColor(colour)
            inner.setAlpha(170)
            outer = QColor(colour)
            outer.setAlpha(0)
            grad.setColorAt(0, inner)
            grad.setColorAt(1, outer)
            painter.fillRect(rect, grad)
        self._paint_card_shadows(painter)
        painter.end()

    def _paint_card_shadows(self, painter):
        """Soft shadow under every visible card: stacked rounded rectangles that
        grow and fade, close to a 40 px blur offset 8 px down."""
        painter.setPen(Qt.NoPen)
        for card in self.findChildren(Card):
            if not card.isVisible():
                continue
            r = QRectF(card.rect()).translated(QPointF(card.mapTo(self, card.rect().topLeft())))
            r.translate(0, 8)
            for k in range(14, 0, -1):
                spread = k * 1.6
                painter.setBrush(QColor(31, 45, 80, int(3.2 * (15 - k) / 14 + 0.4)))
                painter.drawRoundedRect(r.adjusted(-spread, -spread * 0.6, spread, spread), 18 + spread, 18 + spread)


class Card(QFrame):
    """Frosted-glass panel with an optional small uppercase title."""

    def _backdrop_changed(self):
        w = self.parentWidget()
        while w is not None and not isinstance(w, GlassBackground):
            w = w.parentWidget()
        if w is not None:
            w.invalidate()

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self._backdrop_changed()

    def moveEvent(self, event):
        super().moveEvent(event)
        self._backdrop_changed()

    def showEvent(self, event):
        super().showEvent(event)
        self._backdrop_changed()

    def hideEvent(self, event):
        super().hideEvent(event)
        self._backdrop_changed()

    def __init__(self, title=None, parent=None):
        super().__init__(parent)
        self.setObjectName("card")
        # The soft shadow is painted once into the window backdrop (see
        # GlassBackground): a live QGraphicsDropShadowEffect re-blurred the whole
        # card on every value update and cost about a quarter of a CPU core.
        self.body = QVBoxLayout(self)
        self.body.setContentsMargins(18, 16, 18, 16)
        self.body.setSpacing(12)
        if title:
            label = QLabel(title.upper())
            label.setObjectName("cardTitle")
            font = label.font()
            font.setLetterSpacing(QFont.AbsoluteSpacing, 1.2)
            label.setFont(font)
            self.body.addWidget(label)


class StatusPill(QLabel):
    def __init__(self, text="", kind="idle", parent=None):
        super().__init__(text, parent)
        self.setObjectName("pill")
        self.setAlignment(Qt.AlignCenter)
        self.set_state(text, kind)

    def set_state(self, text, kind):
        self.setText(text)
        self.setProperty("kind", kind)
        repolish(self)


class Segmented(QFrame):
    """iOS-style segmented control. Emits the selected option's value."""

    changed = pyqtSignal(object)

    def __init__(self, options, default_index=0, parent=None):
        super().__init__(parent)
        self.setObjectName("segmented")
        layout = QHBoxLayout(self)
        layout.setContentsMargins(3, 3, 3, 3)
        layout.setSpacing(2)
        self._group = QButtonGroup(self)
        self._values = []
        for i, (label, value) in enumerate(options):
            btn = QPushButton(label)
            btn.setObjectName("segment")
            btn.setCheckable(True)
            btn.setCursor(Qt.PointingHandCursor)
            self._group.addButton(btn, i)
            self._values.append(value)
            layout.addWidget(btn)
        self._group.button(default_index).setChecked(True)
        self._group.buttonClicked[int].connect(lambda i: self.changed.emit(self._values[i]))

    def value(self):
        return self._values[self._group.checkedId()]

    def set_value(self, value):
        """Select the option with this value without emitting `changed`."""
        if value in self._values:
            self._group.button(self._values.index(value)).setChecked(True)


class StatCell(QWidget):
    """Small label + monospace value, used for the tool position readout."""

    def __init__(self, name, unit, parent=None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(2)
        top = QLabel(f"{name}  <span style='color:{TEXT_DIM}'>{unit}</span>")
        top.setObjectName("dim")
        self.value = QLabel("—")
        self.value.setObjectName("midValue")
        layout.addWidget(top)
        layout.addWidget(self.value)

    def set_value(self, v):
        self.value.setText("—" if v is None else f"{v:.2f}")


class JointCard(QFrame):
    """One joint: live angle, status hint, and −/+ step buttons."""

    step_clicked = pyqtSignal(int)  # -1 or +1

    def __init__(self, name, absent=False, warning=None, locked=None, parent=None):
        """locked: a reason text when the joint is frozen (hardware fault): the angle
        is still shown, the step buttons stay disabled."""
        super().__init__(parent)
        self.setObjectName("jointCard")
        self.setProperty("absent", absent)
        self.setProperty("moving", False)
        # Expanding horizontally (fills its grid cell nicely at any window width),
        # but only Preferred vertically: it must not balloon into empty space just
        # because a sibling card elsewhere in the window happens to be tall.
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
        self._absent = absent
        self._locked = locked
        self._warning = locked or warning

        layout = QVBoxLayout(self)
        layout.setContentsMargins(14, 12, 14, 12)
        layout.setSpacing(6)

        top = QHBoxLayout()
        name_label = QLabel(name)
        name_label.setObjectName("jointName")
        self.hint = QLabel()
        self.hint.setObjectName("jointHint")
        top.addWidget(name_label)
        top.addStretch(1)
        top.addWidget(self.hint)
        layout.addLayout(top)

        value_row = QHBoxLayout()
        value_row.setSpacing(4)
        self.value = QLabel("—")
        self.value.setObjectName("bigValue")
        unit = QLabel("°")
        unit.setObjectName("unit")
        value_row.addWidget(self.value)
        value_row.addWidget(unit, alignment=Qt.AlignTop)
        value_row.addStretch(1)
        layout.addStretch(1)
        layout.addLayout(value_row)
        layout.addStretch(1)

        buttons = QHBoxLayout()
        buttons.setSpacing(8)
        self.minus_btn = QPushButton("−")
        self.plus_btn = QPushButton("+")
        for btn, sign in ((self.minus_btn, -1), (self.plus_btn, +1)):
            btn.setObjectName("stepBtn")
            btn.setCursor(Qt.PointingHandCursor)
            btn.setEnabled(False)
            if not absent:
                btn.clicked.connect(lambda _=False, s=sign: self.step_clicked.emit(s))
            buttons.addWidget(btn)
        layout.addLayout(buttons)

        if absent:
            self.value.setText("n/a")
            self._set_hint("Not fitted", "")
            for btn in (self.minus_btn, self.plus_btn):
                btn.setToolTip(f"{name} is not fitted on this robot")
        else:
            self.clear_hint()

    def _set_hint(self, text, kind):
        self.hint.setText(text)
        self.hint.setProperty("kind", kind)
        repolish(self.hint)

    def clear_hint(self):
        if self._absent:
            return
        if self._warning:
            self._set_hint(self._warning, "warn")
        else:
            self._set_hint("", "")

    def set_step_size(self, size):
        self.minus_btn.setText(f"−  {size:g}°")
        self.plus_btn.setText(f"+  {size:g}°")

    def set_value(self, v):
        if not self._absent:
            self.value.setText("—" if v is None else f"{v:.2f}")

    def set_steps_enabled(self, enabled):
        if self._locked:
            for btn in (self.minus_btn, self.plus_btn):
                btn.setEnabled(False)
                btn.setToolTip(self._locked)
            return
        if not self._absent:
            self.minus_btn.setEnabled(enabled)
            self.plus_btn.setEnabled(enabled)

    def set_moving(self, moving, target=None):
        self.setProperty("moving", moving)
        repolish(self)
        if moving:
            self._set_hint("moving…" if target is None else f"→ {target:.2f}°", "accent")
        else:
            self.clear_hint()
