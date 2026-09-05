#!/usr/bin/env python3
import tkinter as tk
from tkinter import ttk
import time
import sys
import struct
import socket
import serial
import threading
import subprocess
import os
import json
import csv
import shlex
import shutil
import fcntl
import atexit
import builtins
import math
import queue
import traceback
from collections import deque
from pathlib import Path
from PIL import Image, ImageTk
from src.batchmix_payload import (
    parse_field_color,
    scaled_batchmix_payload_for_water,
)
from src.flow_safety import (
    negative_flow_status,
    negative_totalizer_status,
    positive_drift_status,
)
from src.mopeka_history import (
    history_identity_from_config,
    history_identity_token,
    normalize_history_identity_token,
)

# Version
VERSION_FILE = Path(__file__).with_name("VERSION")
REPO_DIR = Path(__file__).resolve().parent
SIM_MODE = os.environ.get("BBB_SIM_MODE") == "1"
SIM_STATE_DIR = Path(os.environ.get("BBB_SIM_STATE_DIR", REPO_DIR / ".sim-data"))
if SIM_MODE:
    os.environ.setdefault("BBB_SIM_STATE_DIR", str(SIM_STATE_DIR))
SIM_GEOMETRY = os.environ.get("BBB_SIM_GEOMETRY", "1920x1080")
SIM_PUMP_GLIDE_SECONDS = 3.0
SIM_RENDER_SCALE = 1.0
FILL_HISTORY_SOCKET_LIMIT = 20


def _base36_encode(value):
    chars = "0123456789abcdefghijklmnopqrstuvwxyz"
    value = int(value)
    if value == 0:
        return "0"
    sign = ""
    if value < 0:
        sign = "-"
        value = -value
    result = ""
    while value:
        value, remainder = divmod(value, 36)
        result = chars[remainder] + result
    return sign + result


def _geometry_size(geometry):
    try:
        size = geometry.split("+", 1)[0]
        width, height = size.lower().split("x", 1)
        return int(width), int(height)
    except Exception:
        return 1920, 1080


SIM_WINDOW_WIDTH, SIM_WINDOW_HEIGHT = _geometry_size(SIM_GEOMETRY)


def _pi_path(path):
    """Map Pi absolute paths into a local state directory during simulator runs."""
    if SIM_MODE and isinstance(path, (str, os.PathLike)):
        path_str = os.fspath(path)
        pi_prefix = "/home/pi/"
        if path_str.startswith(pi_prefix):
            mapped = SIM_STATE_DIR / path_str[len(pi_prefix):]
            mapped.parent.mkdir(parents=True, exist_ok=True)
            return str(mapped)
    return path


if SIM_MODE:
    SIM_STATE_DIR.mkdir(parents=True, exist_ok=True)
    _real_open = builtins.open

    def _sim_open(file, *args, **kwargs):
        return _real_open(_pi_path(file), *args, **kwargs)

    builtins.open = _sim_open


class _ResilientStream:
    """Make stdout/stderr writes unable to raise or block the caller.

    This process's stdout feeds start_iol_dashboard.sh's log-filter pipe. If
    that filter dies, every print() raises BrokenPipeError (one inside the
    display tick kills the redraw chain: screen frozen while BLE keeps
    serving); if it stalls, a bare print() blocks the whole Tk mainloop.
    Writes go into a bounded queue drained by a daemon thread; when the pipe
    is unusable, lines are counted and dropped - the display must outlive its
    logging.
    """

    def __init__(self, raw, max_queued=2000):
        self._raw = raw
        self._queue = queue.Queue(maxsize=max_queued)
        self._dropped = 0
        self._closing = False
        self._thread = threading.Thread(
            target=self._drain, name="resilient-stream", daemon=True
        )
        self._thread.start()

    def _drain(self):
        while True:
            chunk = self._queue.get()
            if chunk is None:
                break
            try:
                self._raw.write(chunk)
                self._raw.flush()
            except Exception:
                # Broken pipe: swallow and keep draining so writers never
                # see the failure.
                pass

    def write(self, text):
        if not self._closing:
            try:
                self._queue.put_nowait(text)
            except queue.Full:
                self._dropped += 1
        return len(text)

    def flush(self):
        # The drain thread flushes; waiting here would reintroduce the
        # mainloop-blocking failure this class exists to prevent.
        return

    def close_and_drain(self, timeout=2.0):
        """Give queued lines a brief chance to reach the pipe at exit."""
        self._closing = True
        try:
            self._queue.put_nowait(None)
        except queue.Full:
            # Full queue (stalled pipe): sacrifice one queued line so the
            # sentinel fits and the drain still gets its exit signal.
            try:
                self._queue.get_nowait()
                self._queue.put_nowait(None)
            except Exception:
                return
        self._thread.join(timeout)

    def fileno(self):
        return self._raw.fileno()

    def isatty(self):
        return False

    @property
    def dropped(self):
        return self._dropped


if os.environ.get("BBB_DISABLE_RESILIENT_STDOUT") != "1":
    sys.stdout = _ResilientStream(sys.stdout)
    sys.stderr = _ResilientStream(sys.stderr)
    atexit.register(sys.stdout.close_and_drain)
    atexit.register(sys.stderr.close_and_drain)


def _read_local_version():
    try:
        return VERSION_FILE.read_text().strip()
    except Exception:
        return "V1.9.40"


def _read_git_ref_version(git_ref):
    try:
        result = subprocess.run(
            ['git', '-C', '/home/pi/Big-Beautiful-Box', 'show', f'{git_ref}:VERSION'],
            capture_output=True,
            text=True,
            timeout=10,
        )
        if result.returncode == 0 and result.stdout.strip():
            return result.stdout.strip()
    except Exception:
        pass
    return None


VERSION = _read_local_version()
PROGRAM_STARTED_AT = time.time()

# Import configuration
import config
# Module-level fallback: several helpers (should_ignore_menu_ov_bounce,
# adjust_batch_mix_gallons, set_batch_mix_gallons, set_requested_gallons) write to
# this outside serial_listener/socket scope, where it was undefined -> NameError
# aborted the serial read loop (broke the menu OV-bounce guard). serial_listener
# and socket_command_listener still assign their own identical local.
debug_log = config.SERIAL_DEBUG_LOG

if SIM_MODE:
    config.MAIN_LOG_FILE = _pi_path(config.MAIN_LOG_FILE)
    config.SERIAL_DEBUG_LOG = _pi_path(config.SERIAL_DEBUG_LOG)
    config.RELAY_TEST_LOG = _pi_path(config.RELAY_TEST_LOG)
    config.BUTTON_DEBUG_LOG = _pi_path(config.BUTTON_DEBUG_LOG)
    config.FLOW_CONTROL_LOG_FILE = _pi_path(config.FLOW_CONTROL_LOG_FILE)

from src import flow_curve
FLOW_CURVE_OVERRIDE_PATH = _pi_path(config.FLOW_CURVE_OVERRIDE_FILE)
FLOW_CURVE_SAMPLES_PATH = _pi_path(config.FLOW_CURVE_SAMPLES_FILE)
FLOW_CURVE_PROPOSAL_PATH = _pi_path(config.FLOW_CURVE_PROPOSAL_FILE)
active_flow_curve = flow_curve.FlowCurve.factory()
flow_curve_metadata = {"source": "factory", "reason": "startup"}

# Set up rotating loggers
from src.logger import get_main_logger, get_serial_logger, get_button_logger, get_relay_logger
from src.wifi_async import AsyncWifiControl
from src.tank_calibration import (
    compute_point_targets,
    expected_level_in,
    offset_adjustment_inches,
)
main_logger = get_main_logger()
serial_logger = get_serial_logger()
button_logger = get_button_logger()
relay_logger = get_relay_logger()


def log_serial_debug(message):
    """Append a timestamped message to the serial debug log."""
    try:
        with open(config.SERIAL_DEBUG_LOG, 'a') as f:
            f.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')} - {message}\n")
    except Exception:
        pass

def append_debug_log(path, text):
    """Append pre-formatted text to a debug log; I/O errors are swallowed
    so a full/read-only SD card can never abort the calling handler."""
    try:
        with open(path, 'a') as f:
            f.write(text)
    except Exception:
        pass

# Add paths for libraries
sys.path.insert(0, config.RPI_GPIO_PATH)
sys.path.insert(0, config.IOL_HAT_PATH)

try:
    import RPi.GPIO as GPIO
    GPIO_AVAILABLE = True
except ImportError:
    GPIO_AVAILABLE = False
    print("WARNING: RPi.GPIO not available, relay control disabled")

import iolhat


class _SimStatus2:
    def __init__(self, connected=True, powered=True):
        self.pd_in_valid = 1 if connected and powered else 0
        self.transmission_rate = 0x02 if connected and powered else 0
        self.master_cycle_time = 0x20
        self.error = 0x00 if connected and powered else 0x80
        self.power = 1 if powered else 0


class _SimIOLHat:
    LED_GREEN = getattr(iolhat, "LED_GREEN", 2)
    LED_RED = getattr(iolhat, "LED_RED", 1)

    def __init__(self):
        self._lock = threading.RLock()
        self.connected = True
        self.powered = True
        self.totalizer_liters = 0.0
        self.flow_rate_l_per_s = 0.0
        self.temperature_c = 20.0
        self._last_tick = time.time()
        self._flow_start_l_per_s = 0.0
        self._flow_target_l_per_s = 0.0
        self._transition_start = self._last_tick
        self._transition_duration = 0.0
        self._packet_counter = 0
        self._led = self.LED_GREEN

    def _flow_at(self, when):
        if self._transition_duration <= 0:
            return self._flow_target_l_per_s
        progress = (when - self._transition_start) / self._transition_duration
        if progress >= 1:
            return self._flow_target_l_per_s
        if progress <= 0:
            return self._flow_start_l_per_s
        return self._flow_start_l_per_s + (
            self._flow_target_l_per_s - self._flow_start_l_per_s
        ) * progress

    def _tick(self):
        now = time.time()
        elapsed = max(0.0, now - self._last_tick)
        if self.connected and self.powered and elapsed > 0:
            start_flow = self._flow_at(self._last_tick)
            end_flow = self._flow_at(now)
            self.totalizer_liters += ((start_flow + end_flow) / 2.0) * elapsed
            self.flow_rate_l_per_s = end_flow
        else:
            self.flow_rate_l_per_s = self._flow_at(now)
        self._last_tick = now
        if self._transition_duration > 0 and now >= self._transition_start + self._transition_duration:
            self._transition_duration = 0.0
            self._flow_start_l_per_s = self._flow_target_l_per_s
            self.flow_rate_l_per_s = self._flow_target_l_per_s

    def set_flow_gpm(self, gpm):
        with self._lock:
            self._tick()
            flow_l_per_s = max(0.0, float(gpm)) / config.LITERS_PER_SEC_TO_GPM
            self.flow_rate_l_per_s = flow_l_per_s
            self._flow_start_l_per_s = flow_l_per_s
            self._flow_target_l_per_s = flow_l_per_s
            self._transition_start = time.time()
            self._transition_duration = 0.0

    def stop_pump(self, glide_seconds=3.0):
        with self._lock:
            self._tick()
            now = time.time()
            self._flow_start_l_per_s = self.flow_rate_l_per_s
            self._flow_target_l_per_s = 0.0
            self._transition_start = now
            self._transition_duration = max(0.0, float(glide_seconds))

    def get_flow_gpm(self):
        with self._lock:
            self._tick()
            return self.flow_rate_l_per_s * config.LITERS_PER_SEC_TO_GPM

    def reset_totalizer(self):
        with self._lock:
            self.totalizer_liters = 0.0
            self._last_tick = time.time()

    def pd(self, port, index, data_length, callback):
        with self._lock:
            self._tick()
            self._packet_counter = (self._packet_counter + 1) % 256
            if not self.connected or not self.powered:
                return b"\x00" * data_length

            packet = (
                b"\x00\x00\x00\x00"
                + struct.pack(">f", self.totalizer_liters)
                + struct.pack(">f", self.flow_rate_l_per_s)
                + struct.pack(">h", int(round(self.temperature_c * 10)))
                + bytes([0])
            )
            return packet[:data_length].ljust(data_length, b"\x00")

    def power(self, port, status):
        with self._lock:
            self.powered = bool(status)
        return 1

    def led(self, port, color):
        with self._lock:
            self._led = color
        return 1

    def readStatus2(self, port):
        with self._lock:
            return _SimStatus2(self.connected, self.powered)


if SIM_MODE:
    iolhat = _SimIOLHat()

# Global variables
last_totalizer_liters = 0.0
last_signed_totalizer_liters = 0.0
last_flow_rate = 0.0
last_flow_meter_temp_f = None
previous_totalizer_liters = 0.0
connection_error = False
error_message = ""
requested_gallons = config.REQUESTED_GALLONS
serial_connected = False
override_mode = False
last_alert_triggered = False
auto_shutoff_latched = False  # True once the pump-stop relay has fired for the current fill cycle
last_successful_read_time = time.time()
last_flow_read_was_fresh = False  # True only when latest IO-Link data changed
last_fresh_flow_read_time = 0.0  # Monotonic-ish wall time of latest changed IO-Link data
was_flowing = False  # Track if flow was active in previous update (for detecting flow stop)
new_fill_flow_started_at = None  # Time flow first exceeded the new-fill threshold
current_fill_flow_started_at = 0.0  # Epoch when flow started for the in-progress fill segment (0 = none)
new_fill_last_fresh_at = None  # Last fresh high-flow sample during new-fill clearing
new_fill_cycle_cleared = False  # True after prior fill state is cleared for this cycle
colors_are_green = False  # Track if colors have been changed to green
last_reminder_date = None  # Track the last date reminders were shown (YYYY-MM-DD format)
reminders_mode = False  # Track if we're showing reminders
reminders_window = None  # Reference to reminders window
menu_mode = False  # Track if we're in menu mode
last_operator_input_ts = 0.0  # last non-OK/non-OV serial command time (OV phantom diagnostics)
menu_window = None  # Reference to menu window
menu_selected_index = 0  # Currently selected menu item (0=logs, 1=self-test, 2=update, 3=shutdown, 4=reboot, 5=exit-desktop, 6=exit-menu)
menu_buttons = []  # List of menu button widgets
menu_arrows = []  # List of arrow label widgets
menu_daily_label = None  # Reference to daily total label in menu
menu_season_label = None  # Reference to season total label in menu
menu_position_label = None  # Reference to position indicator label
menu_highlight_refresh_pending = False  # Coalesce menu redraws so knob pulses do not stack UI work
menu_displayed_index = None  # Last menu item whose highlight state was painted
menu_ov_guard_until = 0.0  # Suppress OV contact/repeat bounce after screen changes
MENU_OV_GUARD_SECONDS = 2.5
last_serial_ov_toggle_time = 0.0
SERIAL_OV_TOGGLE_DEBOUNCE_SECONDS = 0.5
log_viewer_mode = False  # Track if we're in log viewer
log_viewer_window = None  # Reference to log viewer window
log_viewer_text = None  # Reference to log text widget
fill_history_mode = False  # Track if we're in fill history viewer
fill_history_window = None  # Reference to fill history window
fill_history_text = None  # Reference to fill history text widget
self_test_mode = False  # Track if we're in self-test
self_test_window = None  # Reference to self-test window
full_test_mode = False  # Track if we're in full-test
full_test_window = None  # Reference to full-test window
update_mode = False  # Track if we're in update screen
update_window = None  # Reference to update window
serial_command_received = False  # Track if any serial command has been received (for color change)
exit_confirm_window = None  # Reference to exit confirmation window
exit_confirm_handler = None  # Function to call on confirmation
exit_cancel_handler = None  # Function to call on cancel
reset_season_confirm_window = None  # Reference to reset season confirmation window
reset_season_confirm_handler = None  # Function to call on confirmation
reset_season_cancel_handler = None  # Function to call on cancel
reset_flow_curve_confirm_window = None  # Reference to flow curve reset confirmation
reset_flow_curve_confirm_handler = None  # Function to call on confirmation
reset_flow_curve_cancel_handler = None  # Function to call on cancel
accept_flow_curve_confirm_window = None  # Reference to learned curve accept confirmation
accept_flow_curve_confirm_handler = None  # Function to call on confirmation
accept_flow_curve_cancel_handler = None  # Function to call on cancel
daily_total = 0.0  # Total gallons pumped today
season_total = 0.0  # Total gallons pumped this season (until manually reset)
last_reset_date = None  # Track last daily reset date
last_loads_gallons = []  # Most recent recorded load sizes (newest first)
pending_fill_gallons = 0.0  # Gallons from last fill, waiting for thumbs up confirmation
current_pilot_name = ""  # Name of the most recent pilot (role='pilot') reported by BLE server
current_pilot_connected = False  # True while that pilot is actively connected
last_pilot_disconnect_at = 0.0  # Wall time the pilot last disconnected
# WiFi (RotorLink) pilot, tracked separately from the BLE pilot so a WiFi drop
# never clears a pilot who is still connected over BLE (and vice versa).
# current_pilot_label() prefers the BLE pilot when both are connected.
wifi_pilot_name = ""  # Name of the most recent pilot reported by the RotorLink server
wifi_pilot_connected = False  # True while that WiFi pilot is actively connected
last_wifi_pilot_disconnect_at = 0.0  # Wall time the WiFi pilot last disconnected
PILOT_DISCONNECT_ATTRIBUTION_MAX_SECONDS = 99 * 60  # Stop attributing loads after this gap
# Last reported pilot location per transport ({lat, lon, acc?, ts}) — stamped
# onto recorded loads (| Loc: lat,lon[,acc]) when fresh; see current_pilot_loc().
ble_pilot_loc = None
wifi_pilot_loc = None
PILOT_LOC_MAX_AGE_SECONDS = 15 * 60
pending_fill_requested = 0.0  # Requested gallons from last fill
pending_fill_shutoff_type = ""  # Shutoff type from last fill
pending_fill_flow_gpm = 0.0  # Flow snapshot associated with the completed fill
pending_fill_trigger_threshold = 0.0  # Trigger threshold associated with the completed fill
pending_fill_temp_f = None  # Flow-meter temperature snapshot associated with the completed fill
pending_fill_stop_to_thumb_start_at = 0.0  # Relay activation time for the completed fill
pending_fill_flow_started_at = 0.0  # Epoch when flow first started for the completed fill (0 = unknown)
_sub_threshold_flow_started_at = 0.0  # Sub-4GPM flow tracker (journal evidence of dribbles)
_sub_threshold_flow_peak_lps = 0.0
_sub_threshold_flow_start_gal = 0.0
_sub_threshold_flow_last_log_at = 0.0
pending_fill_flow_ended_at = 0.0  # Epoch when flow stopped for the completed fill (0 = unknown)
last_flowing_rate_l_per_s = 0.0  # Most recent non-zero flow during the current fill
last_trigger_flow_gpm = 0.0  # Flow when auto shutoff triggered
last_trigger_threshold = 0.0  # Threshold when auto shutoff triggered
last_trigger_actual = 0.0  # Actual gallons when auto shutoff triggered
recent_flow_rates_l_per_s = deque(maxlen=config.FLOW_AVERAGING_SAMPLES)
last_heartbeat_time = time.time()  # Last time we received OK heartbeat from switch box
heartbeat_disconnected = False  # Track if heartbeat has timed out
consecutive_identical_raw = 0  # Track byte-for-byte identical reads
last_raw_data = None  # Previous raw bytes for stale detection
last_power_cycle_time = 0         # Timestamp of last IOL power-cycle attempt
iol_power_cycle_in_progress = False  # Flag to prevent overlapping power-cycle threads
override_enabled_time = 0  # Timestamp when override mode was last enabled
iol_io_lock = threading.RLock()  # Single gate for direct IO-Link process-data calls
flow_control_stop_event = threading.Event()
flow_control_thread = None
flow_control_was_flowing = False
flow_control_last_tick = None
flow_control_last_loop_time = time.time()
flow_control_last_error_log_time = 0.0
flow_control_audit_started_at = time.time()
flow_control_audit_polls = 0
flow_control_audit_fresh = 0
flow_control_audit_duplicates = 0
flow_control_audit_flowing_polls = 0
flow_control_audit_flowing_fresh = 0
flow_control_audit_errors = 0
flow_control_audit_loop_ms_total = 0.0
flow_control_audit_loop_ms_max = 0.0
relay_slowdown_watch_active = False
relay_slowdown_alarm_active = False
relay_slowdown_alarm_visible = False
negative_totalizer_alarm_visible = False
relay_slowdown_trigger_time = 0.0
relay_slowdown_trigger_flow_gpm = 0.0
pump_stop_relay_lock = threading.Lock()
pump_stop_pulse_count = 0
last_pump_stop_relay_activated_at = 0.0
pump_stop_fault_hold_active = False
pump_stop_fault_hold_reason = ""
negative_totalizer_fault_active = False
negative_totalizer_fault_reason = ""
negative_totalizer_relay_hold_active = False
last_negative_totalizer_gallons = 0.0
negative_flow_fault_active = False
negative_flow_fault_reason = ""
negative_flow_started_at = 0.0
last_negative_flow_gpm = 0.0
positive_drift_fault_active = False
positive_drift_fault_reason = ""
positive_drift_relay_hold_active = False
positive_drift_low_flow_started_at = 0.0
positive_drift_baseline_liters = 0.0
positive_drift_gallons = 0.0
positive_drift_flow_gpm = 0.0
physical_reset_safety_active = False
physical_reset_gallons_at_event = 0.0
physical_reset_flow_gpm_at_event = 0.0
expected_totalizer_reset_until = 0.0
expected_totalizer_reset_from_gallons = 0.0
physical_reset_safety_window = None
physical_reset_safety_label = None
flow_meter_reconnect_fresh_reads = 0
flow_meter_reconnect_started_at = 0.0
flow_meter_reconnect_last_status_check = 0.0
flow_meter_reconnect_status_ok = False
flow_meter_reconnect_fault_reason = ""
last_trigger_predicted_actual = 0.0
last_trigger_loop_dt_ms = 0.0
# Mix/Fill mode variables
current_mode = "fill"  # Current mode: "fill" or "mix"
fill_requested_gallons = config.REQUESTED_GALLONS  # Preset for fill mode
mix_requested_gallons = 40  # Preset for mix mode (default 40)
mode_indicator_label = None  # Label to display "MIX" in corner
last_status_text = None  # Cache bottom status line to avoid needless redraws
last_daily_total_text = None  # Cache daily total footer text
last_daily_total_mode = None  # Track mode used for daily total rendering
last_flow_rate_text = None  # Cache flow rate footer text
last_flow_rate_mode = None  # Track mode used for flow footer rendering
last_flow_rate_color = None  # Track flow footer color for negative-flow flashing
last_visible_cursor_check = 0.0

# Shared menu order.
MENU_ITEMS = [
    "VIEW LOGS",
    "FILL HISTORY",
    "TANK CALIBRATION",
    "FULL TEST",
    "RESET SEASON",
    "ACCEPT CURVE",
    "FACTORY CURVE",
    "SELF TEST",
    "CAPTURE BUG",
    "SYSTEM UPDATE",
    "SHUTDOWN",
    "REBOOT",
    "EXIT TO DESKTOP",
    "EXIT MENU",
]

# Mopeka tank level display
mopeka1_gallons = 0
mopeka2_gallons = 0
mopeka1_quality = 0
mopeka2_quality = 0
mopeka_connected = False
mopeka_enabled = True
mopeka1_level_mm = 0.0
mopeka2_level_mm = 0.0
mopeka1_level_in = 0.0
mopeka2_level_in = 0.0
mopeka1_has_reading = False
mopeka2_has_reading = False
mopeka1_observed_at = None
mopeka2_observed_at = None
bms_soc = None
bms_voltage = None
bms_has_reading = False
bms_observed_at = None
trailer_sensor_identity_generation = 0
trailer_sensor_identity = None

# Tank calibration workflow state
# Flow stops under this many gallons don't count as a calibration fill —
# meter blips / post-shutoff dribble were starting the settle countdown
# before the pump ever ran.
CALIBRATION_MIN_FILL_GALLONS = 2.0
# An offset correction beyond this is never a real mounting offset — it means
# the sensor wasn't reading during the run (a dead sensor measures 0.00 in at
# every point, which averaged to a +49 in "correction" in the field).
MAX_OFFSET_ADJUSTMENT_IN = 6.0
calibration_mode = False
calibration_window = None
calibration_title_label = None
calibration_body_label = None
calibration_footer_label = None
calibration_hint_label = None
calibration_state = None


# Batch mix data from iPad (cached)
batch_mix_data = None  # Cached JSON data from iPad
batch_mix_overlay = None  # Reference to batch mix overlay frame


def load_flow_curve_state():
    """Load the learned curve override, or fall back to factory values."""
    global active_flow_curve, flow_curve_metadata
    active_flow_curve, flow_curve_metadata = flow_curve.load_curve_override(
        FLOW_CURVE_OVERRIDE_PATH
    )
    status = flow_curve_status_text()
    reason = flow_curve_metadata.get("reason", "")
    msg = f"Loaded flow curve: {status}"
    if reason:
        msg = f"{msg} ({reason})"
    print(msg)
    log_serial_debug(msg)


def flow_curve_status_text():
    """Short field-readable label for the active shutoff curve."""
    if flow_curve_metadata.get("source") == "learned":
        learning = flow_curve_metadata.get("learning", {})
        offset = learning.get("applied_offset_gallons")
        if isinstance(offset, (int, float)):
            return f"Learned {offset:+.2f} gal"
        return "Learned"
    return "Factory"


def flow_curve_proposal_status_text():
    """Short label describing whether a learned curve proposal is waiting."""
    proposal = flow_curve.load_curve_proposal(FLOW_CURVE_PROPOSAL_PATH)
    if not proposal:
        return "No pending curve"
    learning = proposal.get("learning", {})
    offset = learning.get("applied_offset_gallons")
    if isinstance(offset, (int, float)):
        return f"Pending {offset:+.2f} gal"
    return "Pending curve"


def _flow_curve_accept_payload(curve_payload):
    """Build a compact JSON-safe response for an accepted learned curve."""
    learning = curve_payload.get("learning", {}) if isinstance(curve_payload, dict) else {}
    return {
        "current_curve": flow_curve_status_text(),
        "pending_curve": flow_curve_proposal_status_text(),
        "sample_count": curve_payload.get("sample_count", 0) if isinstance(curve_payload, dict) else 0,
        "applied_offset_gallons": learning.get("applied_offset_gallons"),
        "raw_offset_gallons": learning.get("raw_offset_gallons"),
        "accepted_at": curve_payload.get("accepted_at") if isinstance(curve_payload, dict) else None,
    }


def accept_pending_flow_curve(source="Socket"):
    """Activate a pending learned flow curve without opening a UI dialog."""
    proposal = flow_curve.load_curve_proposal(FLOW_CURVE_PROPOSAL_PATH)
    if not proposal:
        return False, {
            "code": "NO_PENDING_CURVE",
            "message": "No learned curve proposal is ready.",
            "current_curve": flow_curve_status_text(),
            "pending_curve": "No pending curve",
        }

    try:
        accepted = flow_curve.accept_curve_proposal(
            FLOW_CURVE_PROPOSAL_PATH,
            FLOW_CURVE_OVERRIDE_PATH,
        )
        load_flow_curve_state()
        with open("/home/pi/fill_calibration.log", "a") as f:
            f.write(
                f"{time.strftime('%Y-%m-%d %H:%M:%S')} | CurveLearning: accepted proposal"
                f" | Source: {source}"
                f" | Offset: {accepted.get('learning', {}).get('applied_offset_gallons', 'unknown')}\n"
            )
        return True, _flow_curve_accept_payload(accepted)
    except Exception as exc:
        with open("/home/pi/fill_calibration.log", "a") as f:
            f.write(
                f"{time.strftime('%Y-%m-%d %H:%M:%S')} | CurveLearning: accept failed"
                f" | Source: {source}"
                f" | Error: {exc}\n"
            )
        return False, {
            "code": "ACCEPT_FAILED",
            "message": str(exc),
            "current_curve": flow_curve_status_text(),
            "pending_curve": flow_curve_proposal_status_text(),
        }


def calculate_trigger_threshold(flow_rate_l_per_s):
    """
    Calculate how many gallons before target to trigger shutoff based on flow rate.
    Uses calibration data to predict coast distance after relay activation.

    Args:
        flow_rate_l_per_s: Current flow rate in liters per second

    Returns:
        Gallons before target to trigger shutoff (predicted coast distance)
    """
    return flow_curve.calculate_trigger_threshold(flow_rate_l_per_s, active_flow_curve)


def get_smoothed_flow_rate():
    """Return a short rolling average of recent flow while flow is active."""
    if not recent_flow_rates_l_per_s:
        return last_flow_rate
    return sum(recent_flow_rates_l_per_s) / len(recent_flow_rates_l_per_s)


def flow_control_enabled():
    """True when the dedicated flow-control loop owns IO-Link shutoff timing."""
    return bool(getattr(config, "FLOW_CONTROL_THREAD_ENABLED", False))


def flow_control_active():
    """True when the configured flow-control thread is currently alive."""
    return bool(
        flow_control_enabled()
        and flow_control_thread is not None
        and flow_control_thread.is_alive()
    )


def flow_read_interval_seconds():
    """Return the expected interval between IO-Link process-data reads."""
    if flow_control_enabled():
        return max(0.01, float(getattr(config, "FLOW_CONTROL_INTERVAL", 0.05)))
    return max(0.01, config.UPDATE_INTERVAL / 1000.0)


def stale_raw_threshold_reads():
    """Convert the stale-data timeout into a read count for the active poll rate."""
    return max(1, int(config.FLOW_METER_TIMEOUT / flow_read_interval_seconds()))


def get_cached_actual_gallons():
    """Return the latest cached totalizer reading without touching IO-Link."""
    return last_totalizer_liters * config.LITERS_TO_GALLONS


def _decode_flow_meter_temp_f(raw_data):
    """Decode Picomag process-data temperature from tenths C to F."""
    if len(raw_data) < 14:
        return None
    temp_c = struct.unpack('>h', raw_data[12:14])[0] / 10.0
    return (temp_c * 9.0 / 5.0) + 32.0


def _format_flow_meter_temp_field(temp_f):
    if temp_f is None:
        return ""
    return f" | Temp: {temp_f:.1f} F"


def _format_stop_to_thumb_field(seconds):
    if seconds is None:
        return ""
    return f" | StopToThumb: {seconds:.1f} s"


def _is_fill_flow_continuation(
    previous_pending_gallons,
    previous_pending_requested,
    previous_flow_started_at,
    requested,
    actual,
    max_extra_gallons=2.0,
):
    """True when the flow segment that just ended is only a post-shutoff
    dribble (or tiny top-off) of a fill ALREADY staged for thumbs-up: same
    requested target, totalizer crept up by at most max_extra_gallons.

    Such a segment must FOLD INTO the pending fill instead of replacing it -
    replacing overwrote a real ~54s flow window (and shutoff type / FlowAtStop
    / stop-to-thumb stats) with the dribble's 0-1s ones, which is exactly the
    "FlowStart equals FlowEnd" corruption seen on TR12's loads (2026-07-04).
    """
    try:
        previous_pending_gallons = float(previous_pending_gallons)
        previous_flow_started_at = float(previous_flow_started_at)
        delta = float(actual) - previous_pending_gallons
    except (TypeError, ValueError):
        return False
    if previous_flow_started_at <= 0 or previous_pending_gallons <= 0:
        return False
    if previous_pending_requested != requested:
        return False
    return -0.001 <= delta <= max_extra_gallons


def _format_flow_window_fields(start_epoch, end_epoch):
    """FlowStart/FlowEnd fields for a fill-history line.

    Each is emitted as a 'YYYY-MM-DD HH:MM:SS' local timestamp (matching the
    leading record timestamp) only when known (epoch > 0). A field is omitted
    when its time is unknown, so the iPad app can detect missing flow times and
    flag the record loudly instead of silently inventing one.
    """
    fields = ""
    if start_epoch:
        fields += " | FlowStart: " + time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(start_epoch))
    if end_epoch:
        fields += " | FlowEnd: " + time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(end_epoch))
    return fields


def _history_named_field(parts, name):
    prefix = f"{name}:"
    for part in parts:
        text = part.strip()
        if text.startswith(prefix):
            return text[len(prefix):].strip()
    return ""


def log_flow_control(message):
    """Append a timestamped flow-control event for shutoff timing audits."""
    try:
        with open(config.FLOW_CONTROL_LOG_FILE, "a") as f:
            f.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')} | {message}\n")
    except Exception:
        pass


def log_relay_event(message):
    """Append a timestamped pump-stop relay event."""
    try:
        with open(config.RELAY_TEST_LOG, "a") as f:
            f.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')} - {message}\n")
    except Exception:
        pass


def _set_pump_stop_output(active, reason):
    """Drive the pump-stop relay output without changing hold/pulse state."""
    if SIM_MODE:
        if active:
            iolhat.stop_pump(SIM_PUMP_GLIDE_SECONDS)
        return True

    if not GPIO_AVAILABLE:
        log_relay_event(f"ERROR: Cannot {'activate' if active else 'release'} relay ({reason}); GPIO unavailable")
        return False

    GPIO.output(config.PUMP_STOP_RELAY_PIN, GPIO.HIGH if active else GPIO.LOW)
    log_relay_event(f"Relay {'HIGH' if active else 'LOW'} ({reason})")
    return True


def set_pump_stop_fault_hold(active, reason="flow meter fault", pulse_on_latch=True):
    """Latch flow-meter fault status and pulse pump stop when the fault starts."""
    global pump_stop_fault_hold_active, pump_stop_fault_hold_reason

    reason = reason or "flow meter fault"
    pulse_fault_stop = False
    with pump_stop_relay_lock:
        if active:
            if pump_stop_fault_hold_active:
                if pump_stop_fault_hold_reason != reason:
                    pump_stop_fault_hold_reason = reason
                    log_flow_control(f"pump_stop_fault_hold_reason | reason={reason}")
                return
            pump_stop_fault_hold_active = True
            pump_stop_fault_hold_reason = reason
            pulse_fault_stop = bool(pulse_on_latch)
            log_flow_control(f"pump_stop_fault_latch_active | reason={reason}")
        else:
            if not pump_stop_fault_hold_active:
                return
            previous_reason = pump_stop_fault_hold_reason
            pump_stop_fault_hold_active = False
            pump_stop_fault_hold_reason = ""
            log_flow_control(f"pump_stop_fault_latch_cleared | reason={previous_reason}")

    if pulse_fault_stop:
        log_flow_control(
            f"pump_stop_fault_momentary_stop | reason={reason}"
            f" | duration={config.PUMP_STOP_DURATION}s"
        )
        start_pump_stop_thread(config.PUMP_STOP_DURATION)


def set_negative_totalizer_relay_hold(active, reason):
    """Hold pump-stop relay HIGH until the negative-totalizer fault clears."""
    global negative_totalizer_relay_hold_active

    reason = reason or "NEGATIVE FLOW METER - RESET REQUIRED"
    with pump_stop_relay_lock:
        if active:
            if negative_totalizer_relay_hold_active:
                return
            negative_totalizer_relay_hold_active = True
            if _set_pump_stop_output(True, f"negative totalizer hold: {reason}"):
                log_flow_control(f"negative_totalizer_relay_hold_active | reason={reason}")
            return

        if not negative_totalizer_relay_hold_active:
            return
        negative_totalizer_relay_hold_active = False
        if pump_stop_pulse_count == 0:
            if positive_drift_relay_hold_active:
                log_flow_control(
                    "negative_totalizer_relay_hold_cleared_positive_hold_active"
                    f" | reason={reason}"
                )
            elif _set_pump_stop_output(False, f"negative totalizer hold cleared: {reason}"):
                log_flow_control(f"negative_totalizer_relay_hold_cleared | reason={reason}")
        else:
            log_flow_control(
                "negative_totalizer_relay_hold_cleared_pending_pulse"
                f" | reason={reason}"
                f" | active_pulses={pump_stop_pulse_count}"
            )


def set_positive_drift_relay_hold(active, reason):
    """Hold pump-stop relay HIGH for positive idle drift unless override is active."""
    global positive_drift_relay_hold_active

    reason = reason or "FLOW METER DRIFT - GALLON RESET REQUIRED"
    with pump_stop_relay_lock:
        if active:
            if override_mode:
                active = False
            else:
                if positive_drift_relay_hold_active:
                    return
                positive_drift_relay_hold_active = True
                if _set_pump_stop_output(True, f"positive drift hold: {reason}"):
                    log_flow_control(f"positive_drift_relay_hold_active | reason={reason}")
                return

        if not positive_drift_relay_hold_active:
            return
        positive_drift_relay_hold_active = False
        if pump_stop_pulse_count == 0:
            if negative_totalizer_relay_hold_active:
                log_flow_control(
                    "positive_drift_relay_hold_cleared_negative_hold_active"
                    f" | reason={reason}"
                )
            elif _set_pump_stop_output(False, f"positive drift hold cleared: {reason}"):
                log_flow_control(f"positive_drift_relay_hold_cleared | reason={reason}")
        else:
            log_flow_control(
                "positive_drift_relay_hold_cleared_pending_pulse"
                f" | reason={reason}"
                f" | active_pulses={pump_stop_pulse_count}"
            )


def _clear_positive_drift_pump_hold_if_owned(reason=None):
    owned_reason = reason or positive_drift_fault_reason
    if (
        owned_reason
        and pump_stop_fault_hold_active
        and pump_stop_fault_hold_reason == owned_reason
    ):
        set_pump_stop_fault_hold(False)


def _reset_positive_drift_monitor(totalizer_liters):
    global positive_drift_low_flow_started_at, positive_drift_baseline_liters
    global positive_drift_gallons, positive_drift_flow_gpm

    positive_drift_low_flow_started_at = 0.0
    positive_drift_baseline_liters = totalizer_liters
    positive_drift_gallons = 0.0
    positive_drift_flow_gpm = 0.0


def update_positive_drift_fault(signed_totalizer_liters, flow_rate_l_per_s):
    """Latch positive totalizer drift under low-flow readings."""
    global positive_drift_fault_active, positive_drift_fault_reason
    global positive_drift_low_flow_started_at, positive_drift_baseline_liters
    global positive_drift_gallons, positive_drift_flow_gpm

    now = time.time()
    signed_gallons = signed_totalizer_liters * config.LITERS_TO_GALLONS
    flow_gpm = flow_rate_l_per_s * config.LITERS_PER_SEC_TO_GPM
    low_flow = flow_gpm < config.FLOW_METER_POSITIVE_DRIFT_LOW_FLOW_GPM

    if negative_totalizer_fault_active or negative_flow_fault_active:
        if positive_drift_fault_active:
            reason = positive_drift_fault_reason
            positive_drift_fault_active = False
            positive_drift_fault_reason = ""
            set_positive_drift_relay_hold(False, reason)
            _clear_positive_drift_pump_hold_if_owned(reason)
        _reset_positive_drift_monitor(signed_totalizer_liters)
        return

    if positive_drift_fault_active and (
        abs(signed_gallons) <= config.FLOW_METER_NEGATIVE_TOTALIZER_CLEAR_GALLONS
        or not low_flow
    ):
        reason = positive_drift_fault_reason
        positive_drift_fault_active = False
        positive_drift_fault_reason = ""
        set_positive_drift_relay_hold(False, reason)
        _clear_positive_drift_pump_hold_if_owned(reason)
        _reset_positive_drift_monitor(signed_totalizer_liters)
        clear_reason = "gallon_reset" if low_flow else "flow_above_threshold"
        log_flow_control(
            "positive_drift_fault_cleared"
            f" | totalizer_gal={signed_gallons:.3f}"
            f" | flow_gpm={flow_gpm:.2f}"
            f" | clear_reason={clear_reason}"
            f" | reason={reason}"
        )
        return

    if positive_drift_low_flow_started_at <= 0 or not low_flow:
        if not positive_drift_fault_active:
            _reset_positive_drift_monitor(signed_totalizer_liters)
        if low_flow:
            positive_drift_low_flow_started_at = now
            positive_drift_baseline_liters = signed_totalizer_liters
        return

    if signed_totalizer_liters < positive_drift_baseline_liters:
        positive_drift_baseline_liters = signed_totalizer_liters

    elapsed = now - positive_drift_low_flow_started_at
    status = positive_drift_status(
        baseline_totalizer_liters=positive_drift_baseline_liters,
        current_totalizer_liters=signed_totalizer_liters,
        flow_rate_l_per_s=flow_rate_l_per_s,
        low_flow_elapsed_seconds=elapsed,
        liters_to_gallons=config.LITERS_TO_GALLONS,
        liters_per_sec_to_gpm=config.LITERS_PER_SEC_TO_GPM,
        low_flow_threshold_gpm=config.FLOW_METER_POSITIVE_DRIFT_LOW_FLOW_GPM,
        drift_threshold_gallons=config.FLOW_METER_POSITIVE_DRIFT_FAULT_GALLONS,
        min_low_flow_seconds=config.FLOW_METER_POSITIVE_DRIFT_SECONDS,
    )
    positive_drift_gallons = max(0.0, status.drift_gallons)
    positive_drift_flow_gpm = status.flow_gpm

    if status.fault:
        should_log_fault = (
            not positive_drift_fault_active
            or positive_drift_fault_reason != status.reason
        )
        positive_drift_fault_active = True
        positive_drift_fault_reason = status.reason
        if override_mode:
            set_positive_drift_relay_hold(False, status.reason)
            _clear_positive_drift_pump_hold_if_owned()
        else:
            set_pump_stop_fault_hold(True, status.reason, pulse_on_latch=False)
            set_positive_drift_relay_hold(True, status.reason)
        if should_log_fault:
            log_flow_control(
                "positive_drift_fault"
                f" | drift_gal={status.drift_gallons:.3f}"
                f" | flow_gpm={status.flow_gpm:.2f}"
                f" | elapsed={elapsed:.2f}"
                f" | threshold={config.FLOW_METER_POSITIVE_DRIFT_FAULT_GALLONS:.3f}"
            )


def update_negative_flow_fault(signed_totalizer_liters, flow_rate_l_per_s):
    """Latch a reset-required fault when signed flow stays negative."""
    global negative_flow_fault_active, negative_flow_fault_reason
    global negative_flow_started_at, last_negative_flow_gpm

    now = time.time()
    signed_gallons = signed_totalizer_liters * config.LITERS_TO_GALLONS
    flow_gpm = flow_rate_l_per_s * config.LITERS_PER_SEC_TO_GPM
    last_negative_flow_gpm = flow_gpm

    if (
        negative_flow_fault_active
        and abs(signed_gallons) <= config.FLOW_METER_NEGATIVE_TOTALIZER_CLEAR_GALLONS
    ):
        reason = negative_flow_fault_reason
        negative_flow_fault_active = False
        negative_flow_fault_reason = ""
        negative_flow_started_at = 0.0
        if not negative_totalizer_fault_active:
            set_negative_totalizer_relay_hold(False, reason)
            set_pump_stop_fault_hold(False)
        log_flow_control(
            "negative_flow_fault_cleared"
            f" | totalizer_gal={signed_gallons:.3f}"
            f" | flow_gpm={flow_gpm:.2f}"
            f" | reason={reason}"
        )
        return

    if flow_gpm > -config.FLOW_METER_NEGATIVE_FLOW_FAULT_GPM:
        if not negative_flow_fault_active:
            negative_flow_started_at = 0.0
        return

    if negative_flow_started_at <= 0:
        negative_flow_started_at = now
        return

    elapsed = now - negative_flow_started_at
    status = negative_flow_status(
        flow_rate_l_per_s=flow_rate_l_per_s,
        negative_flow_elapsed_seconds=elapsed,
        liters_per_sec_to_gpm=config.LITERS_PER_SEC_TO_GPM,
        fault_threshold_gpm=config.FLOW_METER_NEGATIVE_FLOW_FAULT_GPM,
        min_negative_flow_seconds=config.FLOW_METER_NEGATIVE_FLOW_SECONDS,
    )
    last_negative_flow_gpm = status.flow_gpm

    if status.fault:
        should_log_fault = (
            not negative_flow_fault_active
            or negative_flow_fault_reason != status.reason
        )
        negative_flow_fault_active = True
        negative_flow_fault_reason = status.reason
        set_pump_stop_fault_hold(True, status.reason, pulse_on_latch=False)
        set_negative_totalizer_relay_hold(True, status.reason)
        if should_log_fault:
            log_flow_control(
                "negative_flow_fault"
                f" | flow_gpm={status.flow_gpm:.2f}"
                f" | elapsed={elapsed:.2f}"
                f" | threshold={config.FLOW_METER_NEGATIVE_FLOW_FAULT_GPM:.2f}"
            )


def update_negative_totalizer_fault(signed_totalizer_liters):
    """Latch a reset-required fault when the Picomag totalizer drifts negative."""
    global negative_totalizer_fault_active, negative_totalizer_fault_reason
    global last_negative_totalizer_gallons

    status = negative_totalizer_status(
        signed_totalizer_liters=signed_totalizer_liters,
        liters_to_gallons=config.LITERS_TO_GALLONS,
        fault_threshold_gallons=config.FLOW_METER_NEGATIVE_TOTALIZER_FAULT_GALLONS,
        clear_threshold_gallons=config.FLOW_METER_NEGATIVE_TOTALIZER_CLEAR_GALLONS,
    )
    last_negative_totalizer_gallons = status.signed_gallons

    if status.fault:
        should_log_fault = (
            not negative_totalizer_fault_active
            or negative_totalizer_fault_reason != status.reason
        )
        negative_totalizer_fault_active = True
        negative_totalizer_fault_reason = status.reason
        set_pump_stop_fault_hold(True, status.reason, pulse_on_latch=False)
        set_negative_totalizer_relay_hold(True, status.reason)
        if should_log_fault:
            log_flow_control(
                "negative_totalizer_fault"
                f" | signed_gal={status.signed_gallons:.3f}"
                f" | threshold={config.FLOW_METER_NEGATIVE_TOTALIZER_FAULT_GALLONS:.3f}"
            )
    elif negative_totalizer_fault_active and status.reset_clear:
        reason = negative_totalizer_fault_reason
        negative_totalizer_fault_active = False
        negative_totalizer_fault_reason = ""
        if not negative_flow_fault_active:
            set_negative_totalizer_relay_hold(False, reason)
            set_pump_stop_fault_hold(False)
        log_flow_control(
            "negative_totalizer_fault_cleared"
            f" | signed_gal={status.signed_gallons:.3f}"
            f" | reason={reason}"
        )

def update_flow_meter_fault_hold(flow_meter_disconnected=None):
    """Pulse pump stop and latch warning while the flow meter is disconnected or stale."""
    global flow_meter_reconnect_fresh_reads
    global flow_meter_reconnect_started_at, flow_meter_reconnect_last_status_check
    global flow_meter_reconnect_status_ok, flow_meter_reconnect_fault_reason

    if flow_meter_disconnected is None:
        flow_meter_disconnected = (time.time() - last_successful_read_time) > config.FLOW_METER_TIMEOUT

    if negative_totalizer_fault_active or negative_flow_fault_active:
        reason = (
            negative_totalizer_fault_reason
            or negative_flow_fault_reason
            or "NEGATIVE FLOW METER - RESET REQUIRED"
        )
        set_pump_stop_fault_hold(
            True,
            reason,
            pulse_on_latch=False,
        )
        set_negative_totalizer_relay_hold(True, reason)
        return
    if positive_drift_fault_active:
        if override_mode:
            set_positive_drift_relay_hold(False, positive_drift_fault_reason)
            _clear_positive_drift_pump_hold_if_owned()
        else:
            set_pump_stop_fault_hold(
                True,
                positive_drift_fault_reason or "FLOW METER DRIFT - GALLON RESET REQUIRED",
                pulse_on_latch=False,
            )
            set_positive_drift_relay_hold(
                True,
                positive_drift_fault_reason or "FLOW METER DRIFT - GALLON RESET REQUIRED",
            )
            return
    if connection_error:
        flow_meter_reconnect_fresh_reads = 0
        flow_meter_reconnect_started_at = 0.0
        flow_meter_reconnect_status_ok = False
        flow_meter_reconnect_fault_reason = ""
        set_pump_stop_fault_hold(True, error_message or "flow meter connection error")
    elif flow_meter_disconnected:
        flow_meter_reconnect_fresh_reads = 0
        flow_meter_reconnect_started_at = 0.0
        flow_meter_reconnect_status_ok = False
        flow_meter_reconnect_fault_reason = ""
        set_pump_stop_fault_hold(True, "flow meter read timeout")
    elif pump_stop_fault_hold_active:
        now = time.time()
        required_reads = max(1, int(getattr(config, "FLOW_METER_RECONNECT_FRESH_READS", 3)))
        stable_seconds = max(0.0, float(getattr(config, "FLOW_METER_RECONNECT_STABLE_SECONDS", 10.0)))
        if iol_power_cycle_in_progress:
            flow_meter_reconnect_started_at = 0.0
            flow_meter_reconnect_fresh_reads = 0
            flow_meter_reconnect_status_ok = False
            flow_meter_reconnect_fault_reason = ""
        elif now - flow_meter_reconnect_last_status_check >= 1.0:
            flow_meter_reconnect_last_status_check = now
            try:
                flow_meter_reconnect_status_ok, st = _read_iol_status_ok()
                if flow_meter_reconnect_status_ok:
                    if flow_meter_reconnect_started_at <= 0:
                        flow_meter_reconnect_started_at = now
                    flow_meter_reconnect_fresh_reads += 1
                    flow_meter_reconnect_fault_reason = ""
                else:
                    flow_meter_reconnect_started_at = 0.0
                    flow_meter_reconnect_fresh_reads = 0
                    flow_meter_reconnect_fault_reason = _describe_iol_status_fault(st)
                log_flow_control(
                    "flow_meter_reconnect_status"
                    f" | ok={flow_meter_reconnect_status_ok}"
                    f" | pdInValid={st.pd_in_valid}"
                    f" | txRate=0x{st.transmission_rate:02X}"
                    f" | error=0x{st.error:02X}"
                    f" | reason={flow_meter_reconnect_fault_reason or 'healthy'}"
                )
            except Exception as exc:
                flow_meter_reconnect_status_ok = False
                flow_meter_reconnect_started_at = 0.0
                flow_meter_reconnect_fresh_reads = 0
                flow_meter_reconnect_fault_reason = "flow meter status read failed"
                log_flow_control(f"flow_meter_reconnect_status | ok=False | error={exc}")

        stable_elapsed = (
            now - flow_meter_reconnect_started_at
            if flow_meter_reconnect_started_at > 0 else 0.0
        )
        if flow_meter_reconnect_fresh_reads >= required_reads:
            if flow_meter_reconnect_status_ok and stable_elapsed >= stable_seconds:
                flow_meter_reconnect_fresh_reads = 0
                flow_meter_reconnect_started_at = 0.0
                flow_meter_reconnect_status_ok = False
                set_pump_stop_fault_hold(False)
            else:
                set_pump_stop_fault_hold(
                    True,
                    f"waiting for stable flow meter recovery ({stable_elapsed:.1f}/{stable_seconds:.1f}s)",
                )
        else:
            set_pump_stop_fault_hold(
                True,
                flow_meter_reconnect_fault_reason or "waiting for healthy flow meter status",
            )
    else:
        flow_meter_reconnect_fresh_reads = 0
        flow_meter_reconnect_started_at = 0.0
        flow_meter_reconnect_status_ok = False
        flow_meter_reconnect_fault_reason = ""
        set_pump_stop_fault_hold(False)


def _suppress_startup_iol_warning(flow_meter_disconnected=False):
    """Keep startup IO-Link settling quiet on-screen without changing relay safety."""
    grace_seconds = max(0.0, float(getattr(config, "IOL_STARTUP_WARNING_GRACE_SECONDS", 0.0)))
    if grace_seconds <= 0.0 or time.time() - PROGRAM_STARTED_AT > grace_seconds:
        return False
    if last_flow_rate >= config.FLOW_STOPPED_THRESHOLD:
        return False
    return bool(pump_stop_fault_hold_active or connection_error or flow_meter_disconnected)


def start_pump_stop_thread(duration=config.AUTO_ALERT_DURATION):
    """Fire pump-stop relay without blocking the caller."""
    relay_thread = threading.Thread(
        target=pump_stop_relay,
        args=(duration,),
        daemon=True,
    )
    relay_thread.start()

def load_totals():
    """Load daily and season totals from files"""
    global daily_total, season_total, last_reset_date

    # Load daily total
    try:
        with open('/home/pi/daily_total.txt', 'r') as f:
            lines = f.readlines()
            if len(lines) >= 2:
                daily_total = float(lines[0].strip())
                last_reset_date = lines[1].strip()
    except Exception:
        daily_total = 0.0
        last_reset_date = None

    # Load season total
    try:
        with open('/home/pi/season_total.txt', 'r') as f:
            season_total = float(f.read().strip())
    except Exception:
        season_total = 0.0

def save_totals():
    """Save daily and season totals to files"""
    global daily_total, season_total, last_reset_date

    # Save daily total with date
    try:
        with open('/home/pi/daily_total.txt', 'w') as f:
            f.write(f"{daily_total}\n")
            f.write(f"{last_reset_date}\n")
    except Exception as e:
        print(f"Error saving daily total: {e}")

    # Save season total
    try:
        with open('/home/pi/season_total.txt', 'w') as f:
            f.write(f"{season_total}\n")
    except Exception as e:
        print(f"Error saving season total: {e}")


def load_last_load():
    """Load the three most recent recorded actual gallons from fill history."""
    global last_loads_gallons

    try:
        with open('/home/pi/fill_history.log', 'r') as f:
            lines = [line.strip() for line in f if line.strip()]
        last_loads_gallons = []
        for last_line in reversed(lines[-3:]):
            marker = "Actual: "
            start = last_line.index(marker) + len(marker)
            end = last_line.index(" gal", start)
            last_loads_gallons.append(float(last_line[start:end]))
    except Exception:
        last_loads_gallons = []

def load_mode_presets():
    """Load fill and mix mode gallon presets from file"""
    global fill_requested_gallons, mix_requested_gallons, current_mode, batch_mix_data

    try:
        with open('/home/pi/mode_presets.txt', 'r') as f:
            lines = f.readlines()
            if len(lines) >= 3:
                fill_requested_gallons = float(lines[0].strip())
                mix_requested_gallons = float(lines[1].strip())
                current_mode = lines[2].strip()
                if current_mode not in ['fill', 'mix']:
                    current_mode = 'fill'
    except Exception:
        fill_requested_gallons = config.REQUESTED_GALLONS
        mix_requested_gallons = 40
        current_mode = 'fill'

def save_mode_presets():
    """Save fill and mix mode gallon presets to file"""
    global fill_requested_gallons, mix_requested_gallons, current_mode, batch_mix_data

    try:
        with open('/home/pi/mode_presets.txt', 'w') as f:
            f.write(f"{fill_requested_gallons}\n")
            f.write(f"{mix_requested_gallons}\n")
            f.write(f"{current_mode}\n")
    except Exception as e:
        print(f"Error saving mode presets: {e}")

def switch_mode(new_mode):
    """Switch between fill and mix modes"""
    global current_mode, requested_gallons, fill_requested_gallons, mix_requested_gallons
    global mode_indicator_label, colors_are_green, serial_command_received, batch_mix_layout_active
    global thumbs_up_label, thumbs_up_animation_id, override_mode, override_enabled_time
    global last_totalizer_liters, last_flow_rate

    if new_mode == current_mode:
        return  # Already in this mode

    # Save current requested gallons to the current mode
    if current_mode == 'fill':
        fill_requested_gallons = requested_gallons
    else:
        mix_requested_gallons = requested_gallons

    current_actual_gallons = last_totalizer_liters * config.LITERS_TO_GALLONS
    current_flow_gpm = last_flow_rate * 60 * config.LITERS_TO_GALLONS

    if current_mode == 'mix' and new_mode == 'fill' and override_mode:
        override_mode = False
        override_enabled_time = None
        print("Cleared override while switching from MIX to FILL")

    if current_mode == 'mix' and new_mode == 'fill' and current_actual_gallons > 0 and current_flow_gpm < 10:
        root.after(0, lambda: force_flow_reset("mix_to_fill_low_flow"))

    if current_mode == 'fill' and new_mode == 'mix' and current_actual_gallons > 0 and current_flow_gpm < 10:
        root.after(0, lambda: force_flow_reset("fill_to_mix_low_flow"))

    # Switch to new mode and load its preset
    current_mode = new_mode
    if current_mode == 'fill':
        requested_gallons = fill_requested_gallons
    else:
        requested_gallons = mix_requested_gallons

    # Reset color state for new fill
    colors_are_green = False
    serial_command_received = False

    update_mix_mode_indicator()

    if current_mode == 'mix':
        if thumbs_up_animation_id:
            root.after_cancel(thumbs_up_animation_id)
            thumbs_up_animation_id = None
        if thumbs_up_label:
            thumbs_up_label.place_forget()
            _set_thumbs_up_visible(False)

    # Update mode indicator
    if mode_indicator_label:
        if current_mode == 'mix':
            place_mix_mode_indicator()
        else:
            mode_indicator_label.place_forget()

    # Save presets
    save_mode_presets()

    # Update the display
    draw_requested_number(f"{requested_gallons:.0f}", "red")
    update_mopeka_display()
    update_bms_display()
    update_last_load_display()
    update_bms_display()

    print(f"Switched to {current_mode.upper()} mode - requested gallons: {requested_gallons}")

    # Show/hide batch mix overlay based on mode
    update_batch_mix_overlay()

# Track if batch mix layout is active
batch_mix_layout_active = False

# Cache for preventing flicker - only redraw when values change
_last_requested_text = None
_last_requested_color = None
_last_actual_text = None
_last_actual_color = None
_last_batch_requested_text = None
_last_batch_requested_color = None
_last_batch_actual_text = None
_last_batch_actual_color = None

def update_batch_mix_overlay():
    """Update the batch mix screen layout based on mode and data"""
    global batch_mix_layout_active, batch_mix_data, thumbs_up_animation_id

    # Only show batch mix layout in mix mode with data
    if current_mode == "mix" and batch_mix_data is not None:
        place_mix_mode_indicator()
        if thumbs_up_animation_id:
            root.after_cancel(thumbs_up_animation_id)
            thumbs_up_animation_id = None
        if thumbs_up_label:
            thumbs_up_label.place_forget()
            _set_thumbs_up_visible(False)
        if not batch_mix_layout_active:
            activate_batch_mix_layout()
        else:
            refresh_batch_mix_products()
            refresh_batch_mix_totals()
            refresh_batch_mix_tank_levels()
    else:
        update_mix_mode_indicator()
        if batch_mix_layout_active:
            deactivate_batch_mix_layout()
        if current_mode == "mix":
            place_mix_mode_indicator()
        elif mode_indicator_label:
            mode_indicator_label.place_forget()


def clear_batch_mix_screen(reason="clear"):
    """Exit the batch mix product overlay and return to the normal mix screen."""
    global batch_mix_data, thumbs_up_animation_id
    batch_mix_data = None
    if thumbs_up_animation_id:
        root.after_cancel(thumbs_up_animation_id)
        thumbs_up_animation_id = None
    if thumbs_up_label:
        thumbs_up_label.place_forget()
        _set_thumbs_up_visible(False)
    update_batch_mix_overlay()
    msg = f"Batch mix screen cleared: {reason}"
    print(msg)
    log_serial_debug(msg)


def show_batchmix_error(error_msg):
    """Display a BatchMix error message on screen"""
    canvas.delete("batchmix_error")

    width = _canvas_width()
    height = _canvas_height()

    # Red background box
    canvas.create_rectangle(width * 0.1, height * 0.3, width * 0.9, height * 0.5,
                           fill="darkred", outline="red", width=3, tags="batchmix_error")

    # Error title
    canvas.create_text(width // 2, height * 0.35, text="BATCHMIX ERROR",
                      font=("Helvetica", 28, "bold"), fill="white", tags="batchmix_error")

    # Error message
    canvas.create_text(width // 2, height * 0.43, text=error_msg,
                      font=("Helvetica", 20), fill="yellow", tags="batchmix_error")

    # Auto-clear after 5 seconds
    canvas.after(5000, lambda: canvas.delete("batchmix_error"))

def activate_batch_mix_layout():
    """Switch to batch mix screen layout"""
    global batch_mix_layout_active
    global _last_requested_text, _last_requested_color, _last_actual_text, _last_actual_color
    global _last_batch_requested_text, _last_batch_requested_color
    global _last_batch_actual_text, _last_batch_actual_color

    # Clear existing labels and redraw in new positions
    canvas.delete("labels")
    canvas.delete("batchmix")

    _sync_canvas_geometry()
    width = _canvas_width()
    height = _canvas_height()

    # Left 1/3 section - center point
    left_center_x = width // 6

    # Draw "Requested:" label on left side
    canvas.create_text(left_center_x, int(height * 0.13), text="Requested:",
                      font=("Helvetica", 28, "bold"), fill="white", tags="labels")

    # Draw "Actual:" label on left side
    canvas.create_text(left_center_x, int(height * 0.38), text="Actual:",
                      font=("Helvetica", 28, "bold"), fill="white", tags="labels")

    # Separator line above totals (bottom 1/4)
    canvas.create_line(0, int(height * 0.75), width, int(height * 0.75),
                      fill="cyan", width=8, tags="batchmix")

    # Vertical separator between left and right sections
    canvas.create_line(width // 3, 0, width // 3, int(height * 0.75),
                      fill="cyan", width=8, tags="batchmix")

    # Products section title (right 2/3, top area)
    products_x = int(width * 0.54)
    canvas.create_text(products_x, int(height * 0.05), text="PRODUCTS",
                      font=("Helvetica", 32, "bold"), fill="lime", tags="batchmix")
    refresh_batch_mix_tank_levels()

    # Draw products and totals
    refresh_batch_mix_products()
    refresh_batch_mix_totals()

    # Mark layout active and invalidate cached number state so the side layout is forced.
    batch_mix_layout_active = True
    _last_requested_text = None
    _last_requested_color = None
    _last_actual_text = None
    _last_actual_color = None
    _last_batch_requested_text = None
    _last_batch_requested_color = None
    _last_batch_actual_text = None
    _last_batch_actual_color = None

    # Redraw the numbers in new positions
    redraw_numbers_for_batch_mix()

def refresh_batch_mix_totals():
    """Draw/update totals section at bottom of screen"""
    global batch_mix_data

    canvas.delete("totals")

    if batch_mix_data is None:
        return

    _sync_canvas_geometry()
    width = _canvas_width()
    height = _canvas_height()

    # Bottom 1/4 section - totals info
    bottom_y = int(height * 0.85)
    label_y = min(height - 28, bottom_y + 62)
    value_font = ("Helvetica", 77, "bold")
    label_font = ("Helvetica", 42, "bold")

    acres_format = ".2f" if _batch_mix_has_product_rates() else ".1f"
    totals_data = [
        (width * 0.12, f"{batch_mix_data.get('total_acres', 0):{acres_format}}", "ACRES"),
        (width * 0.37, f"{batch_mix_data.get('gallons_per_acre', 0):.1f}", "GAL/AC"),
        (width * 0.62, f"{batch_mix_data.get('total_liquid', 0):.1f}", "TOTAL GAL"),
        (width * 0.87, f"{batch_mix_data.get('water_needed', 0):.1f}", "WATER"),
    ]

    for x, value, label in totals_data:
        # Value - large cyan text
        canvas.create_text(x, bottom_y, text=value, font=value_font,
                          fill="cyan", tags="totals")
        # Label below - smaller gray text
        canvas.create_text(x, label_y, text=label, font=("Helvetica", 24, "bold"),
                          fill="#d0d0d0", tags="totals")

def _mopeka_quality_color(q):
    """Return display color for a Mopeka quality value."""
    if q >= 3:
        return "#00ff00"
    if q >= 2:
        return "#ffff00"
    if q >= 1:
        return "#ff8800"
    return "#ff0000"

def refresh_batch_mix_tank_levels():
    """Draw small tank levels in the BatchMix product header."""
    canvas.delete("batchmix_tanks")

    if current_mode != "mix" or batch_mix_data is None or not mopeka_enabled:
        return

    _sync_canvas_geometry()
    width = _canvas_width()
    height = _canvas_height()
    x = width - 20
    y = int(height * 0.05)
    font = ("Helvetica", 28, "bold")

    if not mopeka_connected:
        canvas.create_text(x, y, text="Tanks: No Signal", font=font,
                          fill="#ff0000", anchor="ne", tags="batchmix_tanks")
        return

    front_text = f"Front: {mopeka1_gallons:.0f}"
    back_text = f"Back: {mopeka2_gallons:.0f}"
    canvas.create_text(x - 220, y, text=front_text, font=font,
                      fill=_mopeka_quality_color(mopeka1_quality),
                      anchor="ne", tags="batchmix_tanks")
    canvas.create_text(x, y, text=back_text, font=font,
                      fill=_mopeka_quality_color(mopeka2_quality),
                      anchor="ne", tags="batchmix_tanks")

def _batch_mix_payload_active():
    """Return True when the visible BatchMix screen should own knob adjustments."""
    return current_mode == "mix" and batch_mix_data is not None

def _format_batch_mix_target(value):
    """Format the water target for the requested-gallons display."""
    try:
        value = float(value)
    except (TypeError, ValueError):
        value = 0.0

    if value == int(value):
        return f"{int(value)}"
    return f"{value:.1f}"

def adjust_batch_mix_gallons(delta, source):
    """Adjust BatchMix gallons and scale all formula amounts from that change."""
    global batch_mix_data, requested_gallons, mix_requested_gallons, colors_are_green

    if not _batch_mix_payload_active():
        return False

    try:
        old_water_needed = float(batch_mix_data.get("water_needed", requested_gallons))
        water_needed = max(1.0, old_water_needed + float(delta))
        batch_mix_data = scaled_batchmix_payload_for_water(batch_mix_data, water_needed)
        water_needed = float(batch_mix_data.get("water_needed", requested_gallons))
        new_acres = float(batch_mix_data.get("total_acres", 0))
    except Exception as exc:
        msg = f"{source}: BatchMix gallon adjust failed: {exc}"
        print(msg)
        try:
            append_debug_log(debug_log, f"{time.strftime('%Y-%m-%d %H:%M:%S')} - {msg}\n")
        except Exception:
            pass
        return True

    requested_gallons = water_needed
    mix_requested_gallons = water_needed
    colors_are_green = False
    save_mode_presets()

    msg = (
        f"{source}: Adjusted BatchMix gallons by {delta:+.0f}, "
        f"water target {water_needed:.1f} gal, acres now {new_acres:.1f}"
    )
    print(msg)
    try:
        append_debug_log(debug_log, f"{time.strftime('%Y-%m-%d %H:%M:%S')} - {msg}\n")
    except Exception:
        pass

    root.after(0, update_batch_mix_overlay)
    root.after(0, lambda value=water_needed: draw_requested_number(_format_batch_mix_target(value), "red"))
    return True


def set_batch_mix_gallons(value, source):
    """Set BatchMix water target and scale all formula amounts to match."""
    global batch_mix_data, requested_gallons, mix_requested_gallons, colors_are_green

    if not _batch_mix_payload_active():
        return None

    try:
        water_target = max(1.0, float(value))
        batch_mix_data = scaled_batchmix_payload_for_water(batch_mix_data, water_target)
        water_needed = float(batch_mix_data.get("water_needed", requested_gallons))
        new_acres = float(batch_mix_data.get("total_acres", 0))
    except Exception as exc:
        msg = f"{source}: BatchMix gallon set failed: {exc}"
        print(msg)
        try:
            append_debug_log(debug_log, f"{time.strftime('%Y-%m-%d %H:%M:%S')} - {msg}\n")
        except Exception:
            pass
        return False

    requested_gallons = water_needed
    mix_requested_gallons = water_needed
    colors_are_green = False
    save_mode_presets()

    msg = (
        f"{source}: Set BatchMix gallons to {water_needed:.1f}, "
        f"acres now {new_acres:.1f}"
    )
    print(msg)
    try:
        append_debug_log(debug_log, f"{time.strftime('%Y-%m-%d %H:%M:%S')} - {msg}\n")
    except Exception:
        pass

    root.after(0, update_batch_mix_overlay)
    root.after(0, lambda value=water_needed: draw_requested_number(_format_batch_mix_target(value), "red"))
    return True


def set_requested_gallons(value, source):
    """Set the active requested-gallons preset from a trusted control path."""
    global requested_gallons, fill_requested_gallons, mix_requested_gallons, colors_are_green

    try:
        target = float(value)
    except (TypeError, ValueError):
        return False, "invalid number"

    if not math.isfinite(target):
        return False, "invalid number"

    if current_mode == "mix":
        batch_mix_result = set_batch_mix_gallons(target, source)
        if batch_mix_result is not None:
            if batch_mix_result:
                return True, requested_gallons
            return False, "batchmix set failed"

    # Clamp to the same sane upper bound the BLE control path enforces (2140 gal).
    # Without an upper clamp, a bad/huge value from the socket or serial path sets
    # an unreachable target and the auto-shutoff (predicted >= requested - threshold)
    # never fires, so the fill relies entirely on a manual pump-stop.
    MAX_REQUESTED_GALLONS = 2140.0
    requested_gallons = min(max(0.0, target), MAX_REQUESTED_GALLONS)
    colors_are_green = False

    if current_mode == "fill":
        fill_requested_gallons = requested_gallons
    else:
        mix_requested_gallons = requested_gallons
    save_mode_presets()

    msg = f"{source}: Set requested gallons to {requested_gallons:.3f}"
    print(msg)
    try:
        append_debug_log(debug_log, f"{time.strftime('%Y-%m-%d %H:%M:%S')} - {msg}\n")
    except Exception:
        pass

    root.after(0, lambda value=requested_gallons: draw_requested_number(_format_batch_mix_target(value), "red"))
    root.after(0, update_batch_mix_overlay)
    return True, requested_gallons

def _format_batch_mix_product_amount(ounces):
    """Format product volume as gallons and ounces for display.

    The ounces are shown to 2 decimal places for a fractional amount (the iPad
    sends amounts at full precision; we no longer round to whole ounces) and as
    a clean integer when whole.
    """
    try:
        total_ounces = max(0.0, float(ounces))
    except (TypeError, ValueError):
        total_ounces = 0.0

    whole_gallons = int(total_ounces // 128)
    remainder_ounces = total_ounces % 128
    if remainder_ounces == int(remainder_ounces):
        oz_text = str(int(remainder_ounces))
    else:
        oz_text = f"{remainder_ounces:.2f}"

    decimal_gallons = total_ounces / 128
    decimal_gallons_text = f"({decimal_gallons:.2f}g)"

    if whole_gallons and remainder_ounces:
        return f"{whole_gallons} gal {oz_text} oz{decimal_gallons_text}"
    if whole_gallons:
        return f"{whole_gallons} gal{decimal_gallons_text}"
    return f"{oz_text} oz{decimal_gallons_text}"

def _format_batch_mix_product_display_amount(prod):
    """Format the product amount from the BatchMix payload."""
    if "amount_oz" in prod:
        return _format_batch_mix_product_amount(prod.get("amount_oz", 0))

    try:
        pounds = max(0.0, float(prod.get("amount_lb", 0)))
    except (TypeError, ValueError):
        pounds = 0.0

    if pounds == int(pounds):
        return f"{int(pounds)} lb"
    return f"{pounds:.2f} lb"

def _format_batch_mix_product_rate(prod):
    """Format the optional product rate from the BatchMix payload."""
    if "rate_per_acre" not in prod or "rate_unit" not in prod:
        return ""

    try:
        rate = max(0.0, float(prod.get("rate_per_acre")))
    except (TypeError, ValueError):
        return ""

    rate_unit = prod.get("rate_unit")
    if not isinstance(rate_unit, str):
        return ""

    rate_unit = rate_unit.strip().lower()
    if rate_unit not in ("oz/ac", "pt/ac", "qt/ac", "gal/ac", "lb/ac"):
        return ""

    if rate == int(rate):
        rate_text = f"{int(rate)}"
    else:
        rate_text = f"{rate:.1f}"
    return f"{rate_text}{rate_unit}"

def _format_batch_mix_product_name(prod):
    """Format product name with optional compact rate."""
    name = prod.get("name", "Unknown")
    return name

def _batch_mix_has_product_rates():
    products = batch_mix_data.get("products", []) if batch_mix_data else []
    if not isinstance(products, list):
        return False
    return any(
        isinstance(product, dict)
        and "rate_per_acre" in product
        and "rate_unit" in product
        for product in products
    )

def _text_width(text, font, anchor="w"):
    test_id = canvas.create_text(0, 0, text=text, font=font,
                                anchor=anchor, tags="temp_measure")
    bbox = canvas.bbox(test_id)
    width = bbox[2] - bbox[0] if bbox else 0
    canvas.delete(test_id)
    return width

def _hex_to_rgb(color):
    """Convert #RRGGBB to an RGB tuple."""
    return tuple(int(color[index:index + 2], 16) for index in (1, 3, 5))

def _contrast_text_color(background):
    """Choose black or white text for readable contrast on a hex background."""
    red, green, blue = _hex_to_rgb(background)
    luminance = (0.299 * red) + (0.587 * green) + (0.114 * blue)
    return "black" if luminance >= 150 else "white"

def _batch_mix_badge_color():
    """Return optional color for the Mix mode badge."""
    colors = batch_mix_data.get("field_colors", []) if batch_mix_data else []
    if not isinstance(colors, list) or not colors:
        return None

    entry = colors[0]
    if not isinstance(entry, dict):
        return None

    color = entry.get("color")
    return parse_field_color(color)

def _striped_mix_badge_image(width, height, first_color, second_color):
    """Return a two-color striped badge image."""
    stripe_width = 18
    first_rgb = _hex_to_rgb(first_color)
    second_rgb = _hex_to_rgb(second_color)
    image = Image.new("RGB", (width, height), first_rgb)
    pixels = image.load()
    for x in range(width):
        stripe_color = second_rgb if (x // stripe_width) % 2 else first_rgb
        for y in range(height):
            pixels[x, y] = stripe_color
    return ImageTk.PhotoImage(image)

def _no_color_mix_badge_image(width, height):
    """Return a white/gray striped badge image for no-color BatchMix payloads."""
    return _striped_mix_badge_image(width, height, "#FFFFFF", "#BEBEBE")

def _contrast_text_color_for_pair(first_color, second_color):
    """Choose black or white text for readable contrast on a striped background."""
    first_rgb = _hex_to_rgb(first_color)
    second_rgb = _hex_to_rgb(second_color)
    blended = "#%02X%02X%02X" % tuple(
        int((first_rgb[index] + second_rgb[index]) / 2) for index in range(3)
    )
    return _contrast_text_color(blended)

def update_mix_mode_indicator():
    """Update the upper-left mix badge from the active BatchMix color."""
    label = globals().get("mode_indicator_label")
    if not label:
        return

    if current_mode != "mix" or batch_mix_data is None:
        label._stripe_image = None
        label.configure(
            text="MIX",
            foreground="cyan",
            background="black",
            image="",
            compound="none",
        )
        return

    _sync_canvas_geometry()
    badge_width = max(180, _canvas_width() // 3)
    badge_height = max(54, int(_canvas_height() * 0.085))
    color_spec = _batch_mix_badge_color()
    if not color_spec:
        label._stripe_image = _no_color_mix_badge_image(badge_width, badge_height)
        label.configure(
            text="NO COLOR MIX",
            foreground="black",
            background="white",
            image=label._stripe_image,
            compound="center",
        )
        return

    if color_spec[0] == "stripe":
        first_color, second_color = color_spec[1], color_spec[2]
        label._stripe_image = _striped_mix_badge_image(
            badge_width, badge_height, first_color, second_color
        )
        label.configure(
            text="MIX",
            foreground=_contrast_text_color_for_pair(first_color, second_color),
            background=first_color,
            image=label._stripe_image,
            compound="center",
        )
        return

    color = color_spec[1]
    label._stripe_image = None
    label.configure(
        text="MIX",
        foreground=_contrast_text_color(color),
        background=color,
        image="",
        compound="none",
    )

def place_mix_mode_indicator():
    """Place the Mix badge for the active mix screen."""
    if not mode_indicator_label:
        return

    if current_mode != "mix":
        mode_indicator_label.place_forget()
        return

    _sync_canvas_geometry()
    update_mix_mode_indicator()
    if batch_mix_data is None:
        mode_indicator_label.place(
            x=10,
            y=10,
            width=150,
            height=58,
            anchor="nw",
        )
        return

    mode_indicator_label.place(
        x=0,
        y=0,
        width=max(180, _canvas_width() // 3),
        height=max(54, int(_canvas_height() * 0.085)),
        anchor="nw",
    )

def refresh_batch_mix_products():
    """Draw/update products list on canvas"""
    global batch_mix_data

    canvas.delete("products")

    if batch_mix_data is None:
        return

    _sync_canvas_geometry()
    width = _canvas_width()
    height = _canvas_height()

    products = batch_mix_data.get("products", [])
    products_x_start = width // 3 + 20
    products_x_end = width - 20

    start_y = int(height * 0.14)
    row_height = int(height * 0.105)  # Height per product row

    for i, prod in enumerate(products[:6]):  # Max 6 products
        y = start_y + (i * row_height)

        # Product name (left side of products area) - auto-scale to fit
        name = _format_batch_mix_product_name(prod)
        rate_text = _format_batch_mix_product_rate(prod)
        rate_suffix = f" ({rate_text})" if rate_text else ""
        max_name_width = (products_x_end - products_x_start) // 2 - 20  # Half the products area

        # Start with larger font, scale down if needed
        font_size = 48
        while font_size >= 22:
            rate_font_size = max(16, int(font_size * 0.62))
            text_width = _text_width(name, ("Helvetica", font_size, "bold"))
            if rate_suffix:
                text_width += _text_width(
                    rate_suffix, ("Helvetica", rate_font_size, "bold")
                )

            if text_width <= max_name_width:
                break
            font_size -= 2

        canvas.create_text(products_x_start + 10, y, text=name,
                          font=("Helvetica", font_size, "bold"), fill="white",
                          anchor="w", tags="products")

        if rate_suffix:
            rate_font_size = max(16, int(font_size * 0.62))
            name_width = _text_width(name, ("Helvetica", font_size, "bold"))
            canvas.create_text(products_x_start + 10 + name_width, y,
                              text=rate_suffix,
                              font=("Helvetica", rate_font_size, "bold"),
                              fill="cyan", anchor="w", tags="products")

        # Amount (right side)
        amount_text = _format_batch_mix_product_display_amount(prod)
        max_amount_width = (products_x_end - products_x_start) // 2 - 20
        amount_font_size = 44
        while amount_font_size >= 22:
            test_id = canvas.create_text(0, 0, text=amount_text,
                                        font=("Helvetica", amount_font_size, "bold"),
                                        anchor="e", tags="temp_measure")
            bbox = canvas.bbox(test_id)
            text_width = bbox[2] - bbox[0] if bbox else 0
            canvas.delete(test_id)

            if text_width <= max_amount_width:
                break
            amount_font_size -= 2

        canvas.create_text(products_x_end - 10, y, text=amount_text,
                          font=("Helvetica", amount_font_size, "bold"), fill="yellow",
                          anchor="e", tags="products")

        # Draw subtle separator line under each product (except last)
        if i < min(len(products), 6) - 1:
            line_y = y + row_height // 2
            canvas.create_line(products_x_start + 5, line_y,
                              products_x_end - 5, line_y,
                              fill="#333355", width=1, tags="products")

    # Easy Mix indicator at bottom of products
    if batch_mix_data.get("easy_mix", False):
        easy_y = start_y + (min(len(products), 6) * row_height) + 10
        canvas.create_text(width * 2 // 3, easy_y, text="EASY MIX",
                          font=("Helvetica", 22, "bold"), fill="lime", tags="products")

def deactivate_batch_mix_layout():
    """Switch back to normal screen layout"""
    global batch_mix_layout_active
    global _last_requested_text, _last_requested_color, _last_actual_text, _last_actual_color
    global _last_batch_requested_text, _last_batch_requested_color
    global _last_batch_actual_text, _last_batch_actual_color

    # Clear batch mix elements
    canvas.delete("batchmix")
    canvas.delete("batchmix_tanks")
    canvas.delete("products")
    canvas.delete("totals")

    # Restore normal labels
    canvas.delete("labels")
    _sync_canvas_geometry()
    center_x = _canvas_width() // 2
    height = _canvas_height()

    canvas.create_text(center_x, int(height * 0.08), text="Requested Gallons:",
                      font=("Helvetica", 36, "bold"), fill="white", tags="labels")
    canvas.create_text(center_x, int(height * 0.45), text="Actual Gallons:",
                      font=("Helvetica", 36, "bold"), fill="white", tags="labels")

    # Invalidate cached number state so normal-mode positions are redrawn.
    _last_requested_text = None
    _last_requested_color = None
    _last_actual_text = None
    _last_actual_color = None
    _last_batch_requested_text = None
    _last_batch_requested_color = None
    _last_batch_actual_text = None
    _last_batch_actual_color = None

    # Leave batch-mix layout before redrawing so normal positioning is used.
    batch_mix_layout_active = False

    # Redraw numbers in normal positions
    redraw_numbers_normal()

# Cache for preventing flicker - only redraw when values change
_last_requested_text = None
_last_requested_color = None
_last_actual_text = None
_last_actual_color = None
_last_batch_requested_text = None
_last_batch_requested_color = None
_last_batch_actual_text = None
_last_batch_actual_color = None

def target_display_color(actual_gallons=None):
    """Return the color used for requested/actual gallons on the main display."""
    if actual_gallons is None:
        actual_gallons = last_totalizer_liters * config.LITERS_TO_GALLONS
    if actual_gallons > requested_gallons + config.WARNING_THRESHOLD:
        return "red"
    return "green" if colors_are_green else "red"

def redraw_numbers_for_batch_mix():
    """Redraw requested/actual numbers in batch mix positions (left 1/3)"""
    global requested_gallons, last_totalizer_liters
    global _last_batch_requested_text, _last_batch_requested_color
    global _last_batch_actual_text, _last_batch_actual_color

    _sync_canvas_geometry()
    width = _canvas_width()
    height = _canvas_height()

    left_center_x = width // 6
    actual_gallons = last_totalizer_liters * config.LITERS_TO_GALLONS
    color = target_display_color(actual_gallons)

    # Requested number - left panel
    req_y = int(height * 0.27)
    req_font = ("Helvetica", 151, "bold")
    req_text = f"{requested_gallons:.0f}"

    if req_text != _last_batch_requested_text or color != _last_batch_requested_color:
        canvas.delete("requested")
        for dx, dy in [(-3,-3), (-3,0), (-3,3), (0,-3), (0,3), (3,-3), (3,0), (3,3)]:
            canvas.create_text(left_center_x+dx, req_y+dy, text=req_text,
                              font=req_font, fill="white", tags="requested")
        canvas.create_text(left_center_x, req_y, text=req_text,
                          font=req_font, fill=color, tags="requested")
        _last_batch_requested_text = req_text
        _last_batch_requested_color = color

    # Actual number - left panel
    act_y = int(height * 0.55)
    act_font = ("Helvetica", 185, "bold")
    act_text = f"{actual_gallons:.1f}"

    if act_text != _last_batch_actual_text or color != _last_batch_actual_color:
        canvas.delete("actual")
        for dx, dy in [(-4,-4), (-4,0), (-4,4), (0,-4), (0,4), (4,-4), (4,0), (4,4)]:
            canvas.create_text(left_center_x+dx, act_y+dy, text=act_text,
                              font=act_font, fill="white", tags="actual")
        canvas.create_text(left_center_x, act_y, text=act_text,
                          font=act_font, fill=color, tags="actual")
        _last_batch_actual_text = act_text
        _last_batch_actual_color = color

def redraw_numbers_normal():
    """Redraw requested/actual numbers in normal centered positions"""
    global requested_gallons, last_totalizer_liters

    actual_gallons = last_totalizer_liters * config.LITERS_TO_GALLONS
    color = target_display_color(actual_gallons)
    draw_requested_number(f"{requested_gallons:.0f}", color)
    draw_actual_number(f"{actual_gallons:.1f}", color)

def add_to_totals(gallons):
    """Add gallons to both daily and season totals"""
    global daily_total, season_total
    daily_total += gallons
    season_total += gallons
    save_totals()


def record_flow_curve_learning_sample():
    """Learn a conservative curve offset from confirmed Auto thumbs-up fills."""
    calibration_log = "/home/pi/fill_calibration.log"
    try:
        sample, reason = flow_curve.make_confirmed_auto_sample(
            requested_gallons=pending_fill_requested,
            actual_gallons=pending_fill_gallons,
            flow_gpm=pending_fill_flow_gpm,
            threshold_gallons=pending_fill_trigger_threshold,
            shutoff_type=pending_fill_shutoff_type,
        )
    except Exception as exc:
        with open(calibration_log, "a") as f:
            f.write(
                f"{time.strftime('%Y-%m-%d %H:%M:%S')} | CurveLearning: error"
                f" | Stage: sample | Error: {exc}\n"
            )
        return

    if sample is None:
        with open(calibration_log, "a") as f:
            f.write(
                f"{time.strftime('%Y-%m-%d %H:%M:%S')} | CurveLearning: skipped"
                f" | Reason: {reason}\n"
            )
        return

    try:
        result = flow_curve.record_learning_sample(
            FLOW_CURVE_SAMPLES_PATH,
            FLOW_CURVE_PROPOSAL_PATH,
            sample,
        )
    except Exception as exc:
        with open(calibration_log, "a") as f:
            f.write(
                f"{time.strftime('%Y-%m-%d %H:%M:%S')} | CurveLearning: error"
                f" | Stage: save | Error: {exc}\n"
            )
        return
    with open(calibration_log, "a") as f:
        f.write(
            f"{time.strftime('%Y-%m-%d %H:%M:%S')} | CurveLearning: accepted"
            f" | Samples: {result['sample_count']}/{result['required_sample_count']}"
        )
        if result.get("proposal_saved"):
            learning = result["learning"]
            f.write(
                f" | ProposalReady: yes"
                f" | Offset: {learning['applied_offset_gallons']:+.3f} gal"
                f" | RawOffset: {learning['raw_offset_gallons']:+.3f} gal"
            )
        f.write("\n")

def reset_daily_total():
    """Reset daily total and log the previous day's total"""
    global daily_total, last_reset_date
    import datetime

    # Log yesterday's total if it was non-zero
    if daily_total > 0:
        try:
            with open('/home/pi/daily_totals_log.log', 'a') as f:
                yesterday = datetime.datetime.now() - datetime.timedelta(days=1)
                f.write(f"{yesterday.strftime('%Y-%m-%d')}: {daily_total:.2f} gallons\n")
        except Exception as e:
            print(f"Error logging daily total: {e}")

    # Reset daily total
    daily_total = 0.0
    last_reset_date = datetime.datetime.now().strftime('%Y-%m-%d')
    save_totals()

def reset_season_total():
    """Reset season total (manual reset from menu)"""
    global season_total
    season_total = 0.0
    save_totals()

def update_totals_display():
    """Update the totals display in the menu (if menu is open)"""
    global menu_daily_label, menu_season_label
    if menu_daily_label and menu_season_label:
        menu_daily_label.config(text=f"Daily: {daily_total:.1f} gal")
        menu_season_label.config(text=f"Season: {season_total:.1f} gal")


def update_last_load_display():
    """Draw the three most recent load sizes in the top-left corner."""
    canvas.delete("last_load")
    if current_mode == "mix":
        return
    if last_loads_gallons:
        load_lines = "\n".join(f"{load:.1f} g" for load in last_loads_gallons[:3])
    else:
        load_lines = "--\n--\n--"
    title_font = ("Helvetica", 72, "bold")
    value_font = ("Helvetica", 96, "bold")
    canvas.create_text(
        20,
        20,
        text="Last Loads:",
        font=title_font,
        fill="cyan",
        anchor="nw",
        tags="last_load",
    )
    canvas.create_text(
        20,
        102,
        text=load_lines,
        font=value_font,
        fill="cyan",
        anchor="nw",
        tags="last_load",
    )


def update_bms_display():
    """Draw battery SOC below the last-load block on the left side."""
    canvas.delete("bms_display")
    if current_mode == "mix":
        return

    if bms_soc is None:
        value_text = "--"
    else:
        value_text = f"{int(round(bms_soc))}%"

    title_font = ("Helvetica", 72, "bold")
    value_font = ("Helvetica", 84, "bold")

    canvas.create_text(
        20,
        505,
        text="SOC:",
        font=title_font,
        fill="cyan",
        anchor="nw",
        tags="bms_display",
    )
    canvas.create_text(
        20,
        587,
        text=value_text,
        font=value_font,
        fill="cyan",
        anchor="nw",
        tags="bms_display",
    )


def update_flow_rate_display(flow_rate_gpm, alert=False):
    """Draw the current flow rate in the bottom-right corner."""
    global last_flow_rate_text, last_flow_rate_mode, last_flow_rate_color

    flow_text = f"Flow:\n{flow_rate_gpm:.1f} GPM"
    if alert:
        flow_color = "red" if int(time.time() * 4) % 2 == 0 else "black"
    else:
        flow_color = "cyan"
    if current_mode == "mix":
        if last_flow_rate_mode != "mix":
            canvas.delete("flow_rate")
        last_flow_rate_mode = "mix"
        last_flow_rate_text = None
        last_flow_rate_color = None
        return

    if (
        last_flow_rate_mode == current_mode
        and last_flow_rate_text == flow_text
        and last_flow_rate_color == flow_color
    ):
        return

    canvas.delete("flow_rate")
    if current_mode == "mix":
        return
    canvas.create_text(
        _canvas_width() - 10,
        _canvas_height() - 10,
        text=flow_text,
        font=("Helvetica", 72, "bold"),
        fill=flow_color,
        anchor="se",
        tags="flow_rate",
    )
    last_flow_rate_text = flow_text
    last_flow_rate_mode = current_mode
    last_flow_rate_color = flow_color


def clear_flow_rate_display():
    """Hide the normal flow footer when a higher-priority warning owns the screen."""
    global last_flow_rate_text, last_flow_rate_mode, last_flow_rate_color

    canvas.delete("flow_rate")
    last_flow_rate_text = None
    last_flow_rate_mode = None
    last_flow_rate_color = None


def update_pilot_status(connected, name):
    """Update the tracked pilot identity reported by the BLE server."""
    global current_pilot_name, current_pilot_connected, last_pilot_disconnect_at
    name = (name or "").strip()
    if connected:
        if name:
            current_pilot_name = name
        current_pilot_connected = True
    else:
        if name:
            current_pilot_name = name
        current_pilot_connected = False
        last_pilot_disconnect_at = time.time()


def update_wifi_pilot_status(connected, name):
    """Update the tracked pilot identity reported by the RotorLink WiFi server."""
    global wifi_pilot_name, wifi_pilot_connected, last_wifi_pilot_disconnect_at
    name = (name or "").strip()
    if connected:
        if name:
            wifi_pilot_name = name
        wifi_pilot_connected = True
    else:
        if name:
            wifi_pilot_name = name
        wifi_pilot_connected = False
        last_wifi_pilot_disconnect_at = time.time()


def update_pilot_loc(source, payload):
    """Parse a PILOT_LOC/WIFI_PILOT_LOC payload ('lat,lon[,acc]')."""
    global ble_pilot_loc, wifi_pilot_loc
    try:
        tokens = [t.strip() for t in str(payload).split(",")]
        loc = {"lat": float(tokens[0]), "lon": float(tokens[1]), "ts": time.time()}
        if len(tokens) >= 3:
            loc["acc"] = float(tokens[2])
    except (TypeError, ValueError, IndexError):
        return
    if source == "wifi":
        wifi_pilot_loc = loc
    else:
        ble_pilot_loc = loc


def current_pilot_loc():
    """Location to stamp on a load — same source precedence as the pilot label
    (connected BLE pilot, then connected WiFi pilot, then whichever pilot
    disconnected most recently); dropped entirely once it goes stale."""
    if current_pilot_connected and current_pilot_name:
        loc = ble_pilot_loc
    elif wifi_pilot_connected and wifi_pilot_name:
        loc = wifi_pilot_loc
    else:
        pairs = []
        if current_pilot_name and last_pilot_disconnect_at > 0:
            pairs.append((last_pilot_disconnect_at, ble_pilot_loc))
        if wifi_pilot_name and last_wifi_pilot_disconnect_at > 0:
            pairs.append((last_wifi_pilot_disconnect_at, wifi_pilot_loc))
        loc = max(pairs, key=lambda p: p[0])[1] if pairs else None
    if not loc:
        return None
    if time.time() - loc.get("ts", 0) > PILOT_LOC_MAX_AGE_SECONDS:
        return None
    return loc


def current_pilot_label():
    """Pilot name to stamp on a load (see record_pending_fill)."""
    if current_pilot_connected and current_pilot_name:
        return current_pilot_name
    if wifi_pilot_connected and wifi_pilot_name:
        return wifi_pilot_name
    # Neither transport has a pilot right now: attribute to whichever pilot
    # disconnected most recently, within the attribution window.
    candidates = []
    if current_pilot_name and last_pilot_disconnect_at > 0:
        candidates.append((last_pilot_disconnect_at, current_pilot_name))
    if wifi_pilot_name and last_wifi_pilot_disconnect_at > 0:
        candidates.append((last_wifi_pilot_disconnect_at, wifi_pilot_name))
    if candidates:
        disconnect_at, name = max(candidates)
        elapsed = time.time() - disconnect_at
        if 0 <= elapsed <= PILOT_DISCONNECT_ATTRIBUTION_MAX_SECONDS:
            minutes = int(elapsed // 60)
            seconds = int(elapsed % 60)
            return f"{name}/disconnected-({minutes}m{seconds}sec)"
    return "Unknown"


def record_pending_fill():
    """Record the pending fill to history log and totals when thumbs up is pressed"""
    global pending_fill_gallons, pending_fill_requested, pending_fill_shutoff_type
    global pending_fill_flow_gpm, pending_fill_trigger_threshold, pending_fill_temp_f
    global pending_fill_stop_to_thumb_start_at
    global pending_fill_flow_started_at, pending_fill_flow_ended_at
    global thumbs_up_label, last_loads_gallons

    # Only record if there's pending fill data
    if pending_fill_gallons > 0:
        fill_log = "/home/pi/fill_history.log"
        calibration_log = "/home/pi/fill_calibration.log"
        recorded_at = time.time()
        timestamp = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(recorded_at))
        stop_to_thumb_seconds = (
            max(0.0, recorded_at - pending_fill_stop_to_thumb_start_at)
            if pending_fill_stop_to_thumb_start_at > 0 else None
        )

        # Write to fill history log
        pilot_label = current_pilot_label()
        pilot_loc = current_pilot_loc()
        loc_field = ""
        if pilot_loc:
            loc_field = f" | Loc: {pilot_loc['lat']:.6f},{pilot_loc['lon']:.6f}"
            if pilot_loc.get('acc') is not None:
                loc_field += f",{pilot_loc['acc']:.1f}"
        with open(fill_log, 'a') as f:
            f.write(
                f"{timestamp} | Requested: {pending_fill_requested:.3f} gal"
                f" | Actual: {pending_fill_gallons:.3f} gal"
                f" | Diff: {pending_fill_gallons - pending_fill_requested:+.3f} gal"
                f" | {pending_fill_shutoff_type}"
                f"{_format_flow_meter_temp_field(pending_fill_temp_f)}"
                f"{_format_stop_to_thumb_field(stop_to_thumb_seconds)}"
                f"{_format_flow_window_fields(pending_fill_flow_started_at, pending_fill_flow_ended_at)}"
                f"{loc_field}"
                f" | Pilot: {pilot_label}\n"
            )

        # Write detailed calibration record with flow snapshot and threshold in one line
        with open(calibration_log, 'a') as f:
            f.write(
                f"{timestamp} | Requested: {pending_fill_requested:.3f} gal"
                f" | Actual: {pending_fill_gallons:.3f} gal"
                f" | Diff: {pending_fill_gallons - pending_fill_requested:+.3f} gal"
                f"{_format_flow_meter_temp_field(pending_fill_temp_f)}"
                f"{_format_stop_to_thumb_field(stop_to_thumb_seconds)}"
                f"{_format_flow_window_fields(pending_fill_flow_started_at, pending_fill_flow_ended_at)}"
                f" | FlowAtStop: {pending_fill_flow_gpm:.1f} GPM"
                f" | Threshold: {pending_fill_trigger_threshold:.3f} gal"
                f" | TriggerActual: {last_trigger_actual:.3f} gal"
                f" | TriggerPredicted: {last_trigger_predicted_actual:.3f} gal"
                f" | TriggerLoopMs: {last_trigger_loop_dt_ms:.1f}"
                f" | Type: {pending_fill_shutoff_type}\n"
            )

        # Add to daily and season totals
        add_to_totals(pending_fill_gallons)
        last_loads_gallons = [pending_fill_gallons] + last_loads_gallons[:2]
        update_last_load_display()
        record_flow_curve_learning_sample()
        print(f"Fill recorded - Actual: {pending_fill_gallons:.3f} gal")
        print(f"Updated totals - Daily: {daily_total:.2f}, Season: {season_total:.2f}")

        # Clear pending fill data
        pending_fill_gallons = 0.0
        pending_fill_requested = 0.0
        pending_fill_shutoff_type = ""
        pending_fill_flow_gpm = 0.0
        pending_fill_trigger_threshold = 0.0
        pending_fill_temp_f = None
        pending_fill_stop_to_thumb_start_at = 0.0
        pending_fill_flow_started_at = 0.0
        pending_fill_flow_ended_at = 0.0

    else:
        print("No pending fill to record")

def handle_thumbs_up_press(source):
    """Accept thumbs up only after flow has stopped."""
    button_log = "/home/pi/button_debug.log"
    if calibration_mode:
        msg = f"Thumbs up ignored from {source}: calibration workflow active"
        print(msg)
        log_serial_debug(msg)
        return

    is_flowing = last_flow_rate >= config.FLOW_STOPPED_THRESHOLD

    if is_flowing:
        msg = (
            f"Thumbs up ignored from {source}: flow still active "
            f"({last_flow_rate:.3f} L/s)"
        )
        print(msg)
        log_serial_debug(msg)
        append_debug_log(button_log, f"{time.strftime('%Y-%m-%d %H:%M:%S')} [DEBUG] {msg}\n")
        return

    change_colors_to_green(from_button=True)
    record_pending_fill()

def pump_stop_relay(duration=config.PUMP_STOP_DURATION):
    """Activate pump stop relay for specified duration"""
    global pump_stop_pulse_count, last_pump_stop_relay_activated_at

    try:
        log_relay_event("")
        log_relay_event("=" * 60)
        log_relay_event(
            f"pump_stop_relay() CALLED | duration={duration} | "
            f"GPIO_AVAILABLE={GPIO_AVAILABLE} | pin={config.PUMP_STOP_RELAY_PIN}"
        )

        with pump_stop_relay_lock:
            pump_stop_pulse_count += 1
            if _set_pump_stop_output(True, f"pulse start {duration}s"):
                last_pump_stop_relay_activated_at = time.time()

        msg = f"Alert relay (GPIO {config.PUMP_STOP_RELAY_PIN}) activated for {duration} seconds"
        print(msg)
        log_relay_event(msg)
        time.sleep(duration)

        relay_released = False
        with pump_stop_relay_lock:
            pump_stop_pulse_count = max(0, pump_stop_pulse_count - 1)
            if pump_stop_pulse_count == 0:
                if negative_totalizer_relay_hold_active or positive_drift_relay_hold_active:
                    hold_reason = (
                        "negative totalizer hold"
                        if negative_totalizer_relay_hold_active else "positive drift hold"
                    )
                    log_relay_event(f"Pulse complete; relay remains HIGH by {hold_reason}")
                else:
                    _set_pump_stop_output(False, "pulse complete")
                    relay_released = True
            else:
                log_relay_event(
                    "Pulse complete; relay remains HIGH "
                    f"(active_pulses={pump_stop_pulse_count})"
                )

        msg = (
            f"Alert relay (GPIO {config.PUMP_STOP_RELAY_PIN}) deactivated"
            if relay_released
            else f"Alert relay (GPIO {config.PUMP_STOP_RELAY_PIN}) pulse complete; relay held HIGH"
        )
        print(msg)
        log_relay_event(msg)
        log_relay_event("=" * 60)
    except Exception as e:
        msg = f"Error controlling relay: {e}"
        print(msg)
        log_relay_event(f"EXCEPTION: {msg}")
        import traceback
        try:
            with open(config.RELAY_TEST_LOG, 'a') as f:
                f.write(traceback.format_exc())
                f.write(f"{'='*60}\n")
        except Exception:
            pass


def get_ip_address():
    """Get the current IP address of the system"""
    try:
        result = subprocess.run(['hostname', '-I'], capture_output=True, text=True, timeout=2)
        ip = result.stdout.strip().split()[0] if result.stdout.strip() else "No IP"
        return ip
    except Exception:
        return "No IP"

def get_assigned_trailer_label():
    """Return the trailer assignment shown in the system menu."""
    try:
        with open(_mopeka_config_path()) as f:
            cfg = json.load(f)
        trailer = cfg.get("assigned_trailer", cfg.get("trailer"))
        if trailer not in (None, ""):
            return f"TR{trailer}"
        display_name = str(cfg.get("display_name") or "").strip()
        if display_name:
            return display_name
    except Exception:
        pass
    return "Unconfigured"

def get_username():
    """Get the current username"""
    try:
        return os.getenv('USER', 'unknown')
    except Exception:
        return 'unknown'

def change_colors_to_green(from_button=False):
    """Change display colors from red to green if within 2 gallons of target, or show thumbs up only if button pressed"""
    global serial_command_received, last_totalizer_liters, requested_gallons, colors_are_green

    button_log = "/home/pi/button_debug.log"
    append_debug_log(button_log, f"{time.strftime('%Y-%m-%d %H:%M:%S')} [DEBUG] change_colors_to_green() called with from_button={from_button}\n")

    if batch_mix_layout_active:
        append_debug_log(button_log, f"{time.strftime('%Y-%m-%d %H:%M:%S')} [DEBUG] Ignoring thumbs-up/green transition while batch mix screen is active\n")
        if thumbs_up_animation_id:
            root.after_cancel(thumbs_up_animation_id)
        if thumbs_up_label:
            thumbs_up_label.place_forget()
            _set_thumbs_up_visible(False)
        return

    # Calculate current actual gallons
    actual_gallons = last_totalizer_liters * config.LITERS_TO_GALLONS
    append_debug_log(button_log, f"{time.strftime('%Y-%m-%d %H:%M:%S')} [DEBUG] Actual: {actual_gallons:.1f}, Requested: {requested_gallons:.0f}, Diff: {abs(actual_gallons - requested_gallons):.1f}\n")

    # Check if within 2 gallons of target
    within_threshold = abs(actual_gallons - requested_gallons) <= 2.0

    if within_threshold:
        append_debug_log(button_log, f"{time.strftime('%Y-%m-%d %H:%M:%S')} [DEBUG] Within 2 gallon threshold! serial_command_received={serial_command_received}\n")
        # Button press always works, auto-alert only works once
        if from_button or not serial_command_received:
            append_debug_log(button_log, f"{time.strftime('%Y-%m-%d %H:%M:%S')} [DEBUG] Proceeding with color change to GREEN...\n")
            if not from_button:
                serial_command_received = True

            # Set the flag so update_dashboard keeps colors green
            colors_are_green = True

            # Change the number colors to green by redrawing
            current_actual = last_totalizer_liters * config.LITERS_TO_GALLONS
            display_color = target_display_color(current_actual)
            draw_requested_number(f"{requested_gallons:.0f}", display_color)
            draw_actual_number(f"{current_actual:.1f}", display_color)
            # Show big thumbs up on the right side and start animation, except on the batch product screen.
            if thumbs_up_label and not batch_mix_layout_active:
                show_thumbs_up(current_actual)
                schedule_flow_reset()
                if thumbs_up_frames:  # Only animate if we have GIF frames
                    animate_thumbs_up()
            source = "button press" if from_button else "auto-alert"
            append_debug_log(
                button_log,
                f"{time.strftime('%Y-%m-%d %H:%M:%S')} Display colors changed to green ({source}, within 2 gallons: {actual_gallons:.1f}/{requested_gallons:.0f})\n"
                f"{time.strftime('%Y-%m-%d %H:%M:%S')} [DEBUG] colors_are_green flag set to True\n")
        else:
            append_debug_log(button_log, f"{time.strftime('%Y-%m-%d %H:%M:%S')} Color change already triggered by auto-alert ({actual_gallons:.1f}/{requested_gallons:.0f})\n")
    else:
        # NOT within threshold
        append_debug_log(button_log, f"{time.strftime('%Y-%m-%d %H:%M:%S')} [DEBUG] NOT within 2 gallon threshold ({actual_gallons:.1f}/{requested_gallons:.0f})\n")

        # If button was pressed, still show thumbs up but keep screen RED
        if from_button:
            append_debug_log(button_log, f"{time.strftime('%Y-%m-%d %H:%M:%S')} [DEBUG] Button pressed but not within threshold - showing thumbs up, keeping RED\n")
            # Show thumbs up but DO NOT change colors to green, except on the batch product screen.
            if thumbs_up_label and not batch_mix_layout_active:
                show_thumbs_up(actual_gallons)
                schedule_flow_reset()
                if thumbs_up_frames:  # Only animate if we have GIF frames
                    animate_thumbs_up()
            append_debug_log(button_log, f"{time.strftime('%Y-%m-%d %H:%M:%S')} Thumbs up shown but screen stays RED (not within 2 gallons: {actual_gallons:.1f}/{requested_gallons:.0f})\n")

def green_button_monitor():
    """Monitor GPIO pin for green button press (active low with pull-up)"""
    button_log = "/home/pi/button_debug.log"

    if not GPIO_AVAILABLE:
        print("GPIO not available, green button monitoring disabled")
        return

    try:
        # Initialize GPIO if not already done
        try:
            GPIO.setmode(GPIO.BCM)
        except Exception:
            pass  # Already initialized

        # Set up green button pin as input with pull-up resistor
        GPIO.setup(config.GREEN_BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        msg = f"Green button monitor started on GPIO {config.GREEN_BUTTON_PIN}"
        print(msg)
        append_debug_log(
            button_log,
            f"\n{'='*60}\n"
            f"{time.strftime('%Y-%m-%d %H:%M:%S')} {msg}\n")

        last_button_state = GPIO.HIGH

        while True:
            # Read current button state
            current_state = GPIO.input(config.GREEN_BUTTON_PIN)

            # Detect button press (transition from HIGH to LOW)
            if last_button_state == GPIO.HIGH and current_state == GPIO.LOW:
                append_debug_log(button_log, f"\n{time.strftime('%Y-%m-%d %H:%M:%S')} *** GREEN BUTTON PRESSED! ***\n")
                print("Green button pressed!")
                # If in reminders mode, dismiss reminders
                if reminders_mode:
                    root.after(0, dismiss_reminders)
                # If in full test mode, mark button test as passed
                elif full_test_mode:
                    if full_test_window and hasattr(full_test_window, 'mark_tested'):
                        root.after(0, lambda: full_test_window.mark_tested('button'))
                        print("Full test: Button test marked as passed")
                else:
                    root.after(0, lambda: handle_thumbs_up_press("GPIO button"))
                # Debounce delay
                time.sleep(0.3)

            last_button_state = current_state
            time.sleep(0.05)  # Check every 50ms

    except Exception as e:
        print(f"Green button monitor error: {e}")

def show_log_viewer():
    """Display log viewer window with button controls"""
    global log_viewer_mode, log_viewer_window, log_viewer_text

    # Submenus should replace the menu, not stack on top of it.
    if menu_window:
        close_menu()
    if log_viewer_window:
        try:
            log_viewer_window.destroy()
        except Exception:
            pass
        log_viewer_window = None

    log_viewer_mode = True
    log_viewer_window = tk.Toplevel()
    log_viewer_window.title("System Logs")
    log_viewer_window.attributes('-fullscreen', True)
    log_viewer_window.configure(bg='black')

    # Title
    title = tk.Label(log_viewer_window, text="SYSTEM LOGS", font=("Helvetica", 32, "bold"),
                     fg="cyan", bg="black")
    title.pack(pady=10)

    # Controls instruction
    controls = tk.Label(log_viewer_window, text="-1=SCROLL UP  +1=SCROLL DOWN  OV=EXIT",
                       font=("Helvetica", 22, "bold"), fg="#ffff00", bg="#0a0a0a")
    controls.pack(pady=2)

    # Create scrolled text widget for logs - HUGE FONT for 7" display
    log_frame = tk.Frame(log_viewer_window, bg='black')
    log_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)

    scrollbar = tk.Scrollbar(log_frame, width=30)
    scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

    # EXTRA LARGE FONT - 24pt bold
    log_viewer_text = tk.Text(log_frame, font=("Courier", 24, "bold"), bg="black", fg="lime",
                             yscrollcommand=scrollbar.set, wrap=tk.WORD)
    log_viewer_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
    scrollbar.config(command=log_viewer_text.yview)

    # Load logs - get last 200 lines for scrolling through history
    try:
        import subprocess

        # Get last 200 lines from system log
        log_viewer_text.insert(tk.END, "=== SYSTEM LOG (last 200 lines) ===\n\n")
        result = subprocess.run(['tail', '-200', '/home/pi/iol_dashboard.log'],
                              capture_output=True, text=True, timeout=2)
        log_viewer_text.insert(tk.END, result.stdout)

        # Get last 200 lines from serial log
        log_viewer_text.insert(tk.END, '\n\n=== SERIAL LOG (last 200 lines) ===\n\n')
        result = subprocess.run(['tail', '-200', '/home/pi/serial_debug.log'],
                              capture_output=True, text=True, timeout=2)
        log_viewer_text.insert(tk.END, result.stdout)

        # Get button debug log if it exists
        log_viewer_text.insert(tk.END, '\n\n=== BUTTON DEBUG LOG (last 100 lines) ===\n\n')
        result = subprocess.run(['tail', '-100', '/home/pi/button_debug.log'],
                              capture_output=True, text=True, timeout=2)
        if result.returncode == 0:
            log_viewer_text.insert(tk.END, result.stdout)
        else:
            log_viewer_text.insert(tk.END, "(No button log found)\n")

        # Scroll to bottom initially
        log_viewer_text.see(tk.END)
    except Exception as e:
        log_viewer_text.insert(tk.END, f"ERROR loading logs:\n{e}")

    # Make read-only but keep enabled for scrolling
    log_viewer_text.config(state=tk.NORMAL)

def log_viewer_scroll_down():
    """Scroll log viewer down"""
    global log_viewer_text
    if log_viewer_text:
        log_viewer_text.yview_scroll(5, "units")  # Scroll down 5 lines (faster scrolling)

def log_viewer_scroll_up():
    """Scroll log viewer up"""
    global log_viewer_text
    if log_viewer_text:
        log_viewer_text.yview_scroll(-5, "units")  # Scroll up 5 lines (faster scrolling)

def close_log_viewer():
    """Close log viewer and return to menu"""
    global log_viewer_mode, log_viewer_window, log_viewer_text
    log_viewer_mode = False
    if log_viewer_window:
        log_viewer_window.destroy()
        log_viewer_window = None
        log_viewer_text = None
    arm_menu_ov_guard()
    show_menu()

def show_fill_history():
    """Display fill history viewer window"""
    global fill_history_mode, fill_history_window, fill_history_text

    # Submenus should replace the menu, not stack on top of it.
    if menu_window:
        close_menu()
    if fill_history_window:
        try:
            fill_history_window.destroy()
        except Exception:
            pass
        fill_history_window = None

    fill_history_mode = True
    fill_history_window = tk.Toplevel()
    fill_history_window.title("Fill History")
    fill_history_window.attributes('-fullscreen', True)
    fill_history_window.configure(bg='black')

    # Title
    title = tk.Label(fill_history_window, text="FILL HISTORY", font=("Helvetica", 32, "bold"),
                     fg="cyan", bg="black")
    title.pack(pady=10)

    # Controls instruction
    controls = tk.Label(fill_history_window, text="-1=SCROLL UP  +1=SCROLL DOWN  OV=EXIT",
                       font=("Helvetica", 22, "bold"), fg="#ffff00", bg="#0a0a0a")
    controls.pack(pady=2)

    # Create scrolled text widget for fill history
    history_frame = tk.Frame(fill_history_window, bg='black')
    history_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)

    scrollbar = tk.Scrollbar(history_frame, width=30)
    scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

    # EXTRA LARGE FONT - 28pt bold for better readability
    fill_history_text = tk.Text(history_frame, font=("Courier", 28, "bold"), bg="black", fg="lime",
                             yscrollcommand=scrollbar.set, wrap=tk.WORD)
    fill_history_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
    scrollbar.config(command=fill_history_text.yview)

    # Load fill history
    try:
        import subprocess

        # Get last 100 fill history entries
        fill_history_text.insert(tk.END, "=== FILL HISTORY (last 100 fills) ===\n\n")
        result = subprocess.run(['tail', '-100', '/home/pi/fill_history.log'],
                              capture_output=True, text=True, timeout=2)
        if result.returncode == 0 and result.stdout:
            fill_history_text.insert(tk.END, result.stdout)
        else:
            fill_history_text.insert(tk.END, "(No fill history found)\n\n")
            fill_history_text.insert(tk.END, "Fill history will appear here after completing fills.\n")

        # Scroll to bottom initially
        fill_history_text.see(tk.END)
    except Exception as e:
        fill_history_text.insert(tk.END, f"ERROR loading fill history:\n{e}")

    # Make read-only but keep enabled for scrolling
    fill_history_text.config(state=tk.NORMAL)

def fill_history_scroll_down():
    """Scroll fill history down"""
    global fill_history_text
    if fill_history_text:
        fill_history_text.yview_scroll(5, "units")  # Scroll down 5 lines

def fill_history_scroll_up():
    """Scroll fill history up"""
    global fill_history_text
    if fill_history_text:
        fill_history_text.yview_scroll(-5, "units")  # Scroll up 5 lines

def close_fill_history():
    """Close fill history and return to menu"""
    global fill_history_mode, fill_history_window, fill_history_text
    fill_history_mode = False
    if fill_history_window:
        fill_history_window.destroy()
        fill_history_window = None
        fill_history_text = None
    arm_menu_ov_guard()
    show_menu()


def _default_calibration_state():
    return {
        "phase": "choose_tank",
        "tank": "front",
        "total_capacity": 1070,
        "target_capacity": 1000,
        "step_size": 10,
        "current_step": 10,
        "points": [],
        "flow_started": False,
        "settle_deadline": None,
        "last_step_actual": 0.0,
        "reading": None,
        "return_phase": None,
        "profile_path": "",
        "profile_error": "",
        # Remote wizard (app-driven) extensions. mode 'full' rebuilds the
        # curve; 'offset' measures against the existing curve and averages the
        # inch error into the sensor's Height Offset. point_targets (when set)
        # replaces the fixed step_size/target_capacity stepping.
        "mode": "full",
        "remote": False,
        "point_targets": None,
        "target_index": 0,
        "max_gallons": 0.0,
        "curve_rows": None,
        "offset_diffs": [],
        "offset_points": [],
        "offset_result": None,
    }


def _selected_tank_reading():
    tank = calibration_state.get("tank", "front")
    if tank == "front":
        return {
            "tank": "front",
            "level_mm": mopeka1_level_mm,
            "level_in": mopeka1_level_in,
            "gallons": mopeka1_gallons,
            "quality": mopeka1_quality,
        }
    return {
        "tank": "back",
        "level_mm": mopeka2_level_mm,
        "level_in": mopeka2_level_in,
        "gallons": mopeka2_gallons,
        "quality": mopeka2_quality,
    }


def _calibration_points_path():
    return "/home/pi/tank_calibration_points.csv"


def _calibration_runs_path():
    return "/home/pi/tank_calibration_runs.json"


def _mopeka_config_path():
    return "/opt/mopeka/mopeka_config.json"


def _read_trailer_sensor_identity_token():
    """Read the durable identity that owns dashboard-cached sensor values."""
    try:
        with open(_mopeka_config_path(), "r") as source:
            cfg = json.load(source)
    except Exception:
        return None
    return history_identity_token(history_identity_from_config(cfg))


def _mopeka_calibration_dir():
    return "/opt/mopeka/calibrations"


def _safe_calibration_profile_key(value):
    key = str(value or "").strip().lower().replace("_", "-")
    return "".join(ch for ch in key if ch.isalnum() or ch == "-")


def _current_tank_calibration_profile_key(tank):
    try:
        with open(_mopeka_config_path(), "r") as f:
            cfg = json.load(f)
    except Exception:
        cfg = {}

    mode = str(cfg.get("box_mode") or "fleet").strip().lower()
    trailer = cfg.get("assigned_trailer", cfg.get("trailer"))
    tank = "back" if tank == "back" else "front"

    if mode != "customer" and trailer not in (None, ""):
        return _safe_calibration_profile_key(f"trailer-{trailer}-{tank}")
    return f"customer-{tank}"


def _current_tank_calibration_profile_path(tank):
    profile_key = _current_tank_calibration_profile_key(tank)
    return os.path.join(_mopeka_calibration_dir(), f"{profile_key}.csv")


def _flow_meter_fault_summary():
    reason = negative_totalizer_fault_reason or (
        negative_flow_fault_reason or (
            positive_drift_fault_reason or (
                pump_stop_fault_hold_reason if pump_stop_fault_hold_active else ""
            )
        )
    )
    if negative_totalizer_fault_active:
        return True, "negative_totalizer", reason
    if negative_flow_fault_active:
        return True, "negative_flow", reason
    if positive_drift_fault_active:
        return True, "positive_drift", reason
    if reason:
        return True, "flow_meter", reason
    return False, "", ""


def _build_dashboard_state_snapshot():
    """Build a compact state snapshot for BLE/iPad clients."""
    actual_gallons = last_totalizer_liters * config.LITERS_TO_GALLONS
    flow_gpm = last_flow_rate * config.LITERS_PER_SEC_TO_GPM
    flow_meter_connected = (time.time() - last_successful_read_time) <= config.FLOW_METER_TIMEOUT
    switch_box_connected = bool(serial_connected and not heartbeat_disconnected)
    fill_pending = pending_fill_gallons > 0
    can_confirm_fill = bool(fill_pending and last_flow_rate < config.FLOW_STOPPED_THRESHOLD)
    flow_fault_active, flow_fault_code, flow_fault_reason = _flow_meter_fault_summary()

    return {
        "version": VERSION,
        # Age of the last display redraw tick. Lets remote clients (app /
        # maintenance shell) see a frozen box screen that the box itself
        # cannot report any other way.
        "display_tick_age_s": round(max(0.0, time.time() - last_dashboard_tick_at), 1),
        "requested_gal": round(requested_gallons, 3),
        "actual_gal": round(actual_gallons, 3),
        "flow_gpm": round(flow_gpm, 2),
        "mode": current_mode,
        "override": bool(override_mode),
        "thumbs_visible": bool(thumbs_up_visible),
        "fill_pending": bool(fill_pending),
        "can_confirm_fill": bool(can_confirm_fill),
        "pending_fill": {
            "requested_gal": round(pending_fill_requested, 3),
            "actual_gal": round(pending_fill_gallons, 3),
            "flow_gpm": round(pending_fill_flow_gpm, 1),
            "shutoff_type": pending_fill_shutoff_type,
            "flow_meter_temp_f": None if pending_fill_temp_f is None else round(pending_fill_temp_f, 1),
            "thumbs_status": "pending",
        } if fill_pending else None,
        "colors_green": bool(colors_are_green),
        "pump_stop_latched": bool(auto_shutoff_latched),
        "relay_slowdown_alarm": bool(relay_slowdown_alarm_active),
        "flow_meter_connected": bool(flow_meter_connected),
        "negative_totalizer_fault": bool(negative_totalizer_fault_active),
        "negative_totalizer_gal": round(last_negative_totalizer_gallons, 3),
        "negative_flow_fault": bool(negative_flow_fault_active),
        "negative_flow_gpm": round(last_negative_flow_gpm, 2),
        "positive_drift_fault": bool(positive_drift_fault_active),
        "positive_drift_gal": round(positive_drift_gallons, 3),
        "positive_drift_flow_gpm": round(positive_drift_flow_gpm, 2),
        "flow_fault_active": bool(flow_fault_active),
        "flow_fault_code": flow_fault_code,
        "flow_meter_fault_reason": flow_fault_reason,
        "switch_box_connected": bool(switch_box_connected),
        "bms_soc": None if not bms_has_reading or bms_soc is None else int(round(bms_soc)),
        "bms_voltage": None if not bms_has_reading or bms_voltage is None else round(bms_voltage, 2),
        "bms_has_reading": bool(bms_has_reading),
        "bms_last_update": bms_observed_at if bms_has_reading else None,
        "daily_total_gal": round(daily_total, 3),
        "front_tank_gal": round(mopeka1_gallons, 1) if mopeka1_has_reading else None,
        "back_tank_gal": round(mopeka2_gallons, 1) if mopeka2_has_reading else None,
        "front_tank_quality": int(mopeka1_quality) if mopeka1_has_reading else None,
        "back_tank_quality": int(mopeka2_quality) if mopeka2_has_reading else None,
        # Raw sensor level so WiFi clients can show inches like the BLE
        # sensor characteristics do (level_in is offset-compensated).
        "front_tank_mm": round(float(mopeka1_level_mm), 1),
        "back_tank_mm": round(float(mopeka2_level_mm), 1),
        "front_tank_in": round(float(mopeka1_level_in), 2),
        "back_tank_in": round(float(mopeka2_level_in), 2),
        "front_tank_has_reading": bool(mopeka1_has_reading),
        "back_tank_has_reading": bool(mopeka2_has_reading),
        "front_tank_last_update": mopeka1_observed_at if mopeka1_has_reading else None,
        "back_tank_last_update": mopeka2_observed_at if mopeka2_has_reading else None,
        "trailer_sensor_identity_generation": trailer_sensor_identity_generation,
        "trailer_sensor_identity": trailer_sensor_identity,
        "mopeka_enabled": bool(mopeka_enabled),
        "mopeka_connected": bool(mopeka_connected),
        "last_loads_gal": [round(load, 3) for load in last_loads_gallons[:3]],
        "current_curve": flow_curve_status_text(),
        "pending_curve": flow_curve_proposal_status_text(),
        "calibration": _calibration_snapshot(),
    }


def _calibration_snapshot():
    """Remote view of the calibration wizard (None when not running) — the
    app's wizard UI renders this straight from STATE_JSON."""
    if not calibration_mode or not calibration_state:
        return None
    st = calibration_state
    targets = st.get("point_targets") or []
    settle_remaining = None
    if st.get("phase") == "settling" and st.get("settle_deadline"):
        settle_remaining = max(0, int(st["settle_deadline"] - time.time()))
    # Frozen settled reading at review/complete; LIVE sensor reading during the
    # other phases so the operator can watch the inches while filling/settling.
    reading = st.get("reading")
    if not reading and st.get("phase") in ("confirm_empty", "wait_for_fill", "settling"):
        reading = _selected_tank_reading()
    # Offset mode review: what the existing curve EXPECTS the level to be at
    # the gallons actually pumped, so the app can show original vs measured
    # before the point is accepted.
    expected_in = None
    if (
        st.get("mode") == "offset"
        and st.get("phase") == "review"
        and reading
        and st.get("curve_rows")
        and st.get("last_step_actual") is not None
    ):
        try:
            expected_in = round(expected_level_in(st["curve_rows"], float(st["last_step_actual"])), 2)
        except Exception:
            expected_in = None
    return {
        "mode": st.get("mode", "full"),
        "phase": st.get("phase"),
        "tank": st.get("tank"),
        "step_index": int(st.get("target_index", 0)),
        "points_total": len(targets) if targets else None,
        # Before the first fill current_step still holds the legacy default;
        # the target list is authoritative when present.
        "target_gallons": round(float(
            targets[min(int(st.get("target_index", 0)), len(targets) - 1)]
            if targets else (st.get("current_step") or 0)
        ), 1),
        "settle_remaining": settle_remaining,
        "actual_gallons": (
            None if st.get("last_step_actual") is None
            else round(float(st["last_step_actual"]), 1)
        ),
        "points_recorded": (
            len(st.get("offset_points") or []) if st.get("mode") == "offset"
            else len(st.get("points") or [])
        ),
        "reading": None if not reading else {
            "mm": round(float(reading.get("level_mm") or 0), 1),
            "in": round(float(reading.get("level_in") or 0), 2),
            "gal": round(float(reading.get("gallons") or 0), 1),
            "q": int(reading.get("quality") or 0),
            **({"ex": expected_in} if expected_in is not None else {}),
        },
        "offset_result": st.get("offset_result"),
        "error": st.get("profile_error") or None,
    }


def _save_calibration_run():
    if not calibration_state:
        return
    if not calibration_state.get("points") and not calibration_state.get("offset_points"):
        return

    run_record = {
        "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
        "tank": calibration_state["tank"],
        "mode": calibration_state.get("mode", "full"),
        "remote": bool(calibration_state.get("remote")),
        "total_capacity": calibration_state["total_capacity"],
        "target_capacity": calibration_state["target_capacity"],
        "step_size": calibration_state["step_size"],
        "points": calibration_state["points"],
        "offset_points": calibration_state.get("offset_points") or [],
    }

    runs_path = _calibration_runs_path()
    try:
        if os.path.exists(runs_path):
            with open(runs_path, "r") as f:
                runs = json.load(f)
                if not isinstance(runs, list):
                    runs = []
        else:
            runs = []
    except Exception:
        runs = []

    runs.append(run_record)
    with open(runs_path, "w") as f:
        json.dump(runs, f, indent=2)

    points_path = _calibration_points_path()
    write_header = not os.path.exists(points_path)
    with open(points_path, "a") as f:
        if write_header:
            f.write(
                "timestamp,tank,total_capacity,target_capacity,step_target_gallons,"
                "actual_total_gallons,level_mm,level_in,display_gallons,quality\n"
            )
        for point in calibration_state["points"]:
            f.write(
                f"{run_record['timestamp']},{run_record['tank']},{run_record['total_capacity']},"
                f"{run_record['target_capacity']},{point['step_target_gallons']},"
                f"{point['actual_total_gallons']:.3f},{point['level_mm']:.1f},"
                f"{point['level_in']:.2f},{point['display_gallons']:.1f},{point['quality']}\n"
            )


def _append_calibration_point(actual_gallons, step_target_gallons=None):
    reading = calibration_state.get("reading") or _selected_tank_reading()
    calibration_state["points"].append({
        "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
        "step_target_gallons": (
            actual_gallons if step_target_gallons is None else step_target_gallons
        ),
        "actual_total_gallons": actual_gallons,
        "level_mm": reading["level_mm"],
        "level_in": reading["level_in"],
        "display_gallons": reading["gallons"],
        "quality": reading["quality"],
    })


def _record_offset_point(actual_gallons):
    """Offset mode: compare the settled reading to the EXISTING curve at the
    same gallons and keep the inch difference. The measured level already
    includes the current Height Offset, so the averaged difference is the
    correction to ADD to it."""
    reading = calibration_state.get("reading") or _selected_tank_reading()
    expected_in = expected_level_in(calibration_state["curve_rows"], actual_gallons)
    diff_in = expected_in - float(reading["level_in"])
    calibration_state["offset_diffs"].append(diff_in)
    calibration_state["offset_points"].append({
        "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
        "gallons": float(actual_gallons),
        "measured_in": float(reading["level_in"]),
        "expected_in": round(expected_in, 3),
        "diff_in": round(diff_in, 3),
        "level_mm": reading["level_mm"],
        "quality": reading["quality"],
    })


def _compute_offset_result():
    """Average the offset diffs against the sensor's current Height Offset.
    Pure read — used both for the pre-apply preview (complete phase) and the
    actual apply, so the number the operator confirms is the number saved."""
    tank = calibration_state["tank"]
    sensor_id = _selected_tank_sensor_id(tank)
    if not sensor_id:
        raise ValueError(f"no {tank} sensor configured")
    adjustment = offset_adjustment_inches(calibration_state["offset_diffs"])
    current = _read_sensor_height_offset(sensor_id)
    return {
        "sensor_id": sensor_id,
        "previous_offset_in": current,
        "adjustment_in": adjustment,
        "new_offset_in": round(current + adjustment, 2),
        "points": len(calibration_state["offset_diffs"]),
    }


def _apply_offset_calibration():
    """Persist the averaged Height Offset. Returns a human summary string
    (shown where the full mode shows the profile path)."""
    result = _compute_offset_result()
    if abs(result["adjustment_in"]) > MAX_OFFSET_ADJUSTMENT_IN:
        raise ValueError(
            f"Correction {result['adjustment_in']:+.1f} in is out of range "
            f"(max ±{MAX_OFFSET_ADJUSTMENT_IN:.0f} in) — the sensor likely "
            "wasn't reading during the run. Nothing was saved."
        )
    _write_sensor_height_offset(result["sensor_id"], result["new_offset_in"])
    calibration_state["offset_result"] = result
    return "offset {previous_offset_in:+.2f} -> {new_offset_in:+.2f} in ({sensor_id})".format(**result)


def _apply_tank_calibration_profile():
    if not calibration_state or len(calibration_state.get("points", [])) < 2:
        raise ValueError("Need at least empty plus one fill point")

    tank = calibration_state["tank"]
    profile_path = _current_tank_calibration_profile_path(tank)
    os.makedirs(os.path.dirname(profile_path), exist_ok=True)

    if os.path.exists(profile_path):
        backup_path = f"{profile_path}.backup-{time.strftime('%Y%m%d_%H%M%S')}"
        with open(profile_path, "r") as src, open(backup_path, "w") as dst:
            dst.write(src.read())

    rows = []
    for point in calibration_state["points"]:
        rows.append({
            "tank_level_in": float(point["level_in"]),
            "gallons": float(point["actual_total_gallons"]),
            "tank_size": float(calibration_state["total_capacity"]),
        })
    rows.sort(key=lambda row: row["tank_level_in"], reverse=True)

    tmp_path = f"{profile_path}.tmp"
    with open(tmp_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["Tank Level (in)", "Gallons", "Tank Size (gal)"])
        for row in rows:
            writer.writerow([row["tank_level_in"], row["gallons"], row["tank_size"]])
    os.replace(tmp_path, profile_path)
    return profile_path


def _refresh_calibration_window():
    if not calibration_window or not calibration_state:
        return

    phase = calibration_state["phase"]
    tank_label = calibration_state["tank"].title()
    title = "TANK CALIBRATION"
    body = ""
    footer = ""
    hint = ""

    if phase == "choose_tank":
        body = f"Select Tank\n\n{tank_label} Tank"
        footer = "+1/-1 = CHANGE   OV = CONFIRM   PS = CANCEL"
        hint = "Choose which trailer tank to calibrate."
    elif phase == "set_total":
        body = f"{tank_label} Tank\n\nTotal Capacity\n{calibration_state['total_capacity']} gal"
        footer = "+10/-10, +1/-1 = ADJUST   OV = NEXT   PS = BACK"
        hint = "Set the full physical tank capacity."
    elif phase == "set_target":
        body = (
            f"{tank_label} Tank\n\nCalibration Endpoint\n"
            f"{calibration_state['target_capacity']} gal"
        )
        footer = "+10/-10, +1/-1 = ADJUST   OV = NEXT   PS = BACK"
        hint = "Set the usable gallons to calibrate up to."
    elif phase == "confirm_empty":
        body = f"{tank_label} Tank\n\nConfirm this tank is completely empty."
        footer = "OV = YES, START   PS = BACK"
        hint = "The workflow will fill in 10 gallon increments."
    elif phase == "wait_for_fill":
        step = calibration_state["current_step"]
        body = (
            f"{tank_label} Tank\n\nTarget Step: {step} gal\n\n"
            "Start the pump.\nBBB will stop flow automatically."
        )
        footer = "PS = ABORT"
        hint = "Waiting for flow to start."
        if calibration_state.get("flow_started"):
            hint = "Filling..."
    elif phase == "settling":
        remaining = max(0, int(calibration_state["settle_deadline"] - time.time()))
        body = (
            f"{tank_label} Tank\n"
            f"{calibration_state['current_step']} gal reached.\n"
            "Waiting for Mopeka to settle\n"
            f"{remaining} sec remaining"
        )
        footer = "PS = ABORT"
        hint = "Do not disturb the tank during the settle period."
    elif phase == "review":
        reading = calibration_state.get("reading") or _selected_tank_reading()
        body = (
            f"{tank_label} Tank\n\nStep: {calibration_state['current_step']} gal\n"
            f"Actual: {calibration_state['last_step_actual']:.1f} gal\n"
            f"Mopeka: {reading['level_mm']:.1f} mm / {reading['level_in']:.2f} in\n"
            f"Display: {reading['gallons']:.0f} gal   Quality: {reading['quality']}"
        )
        footer = "OV = SAVE/NEXT   +1 = REREAD   -1 = WAIT 2 MORE MIN   PS = ABORT"
        hint = "Confirm the Mopeka reading before continuing."
    elif phase == "complete":
        if calibration_state.get("mode") == "offset" and calibration_state.get("offset_result"):
            r = calibration_state["offset_result"]
            body = (
                f"{tank_label} Tank Offset Measured\n\n"
                f"{r['points']} points, avg correction {r['adjustment_in']:+.2f} in\n"
                f"Height Offset {r['previous_offset_in']:+.2f} -> {r['new_offset_in']:+.2f} in\n\n"
                "Save this offset to the sensor?"
            )
            footer = "OV = SAVE OFFSET   PS = RETURN"
            hint = r["sensor_id"]
        else:
            profile_path = _current_tank_calibration_profile_path(calibration_state["tank"])
            body = (
                f"{tank_label} Tank Calibration Complete\n\n"
                f"Saved {len(calibration_state['points'])} calibration points\n"
                f"through {calibration_state['target_capacity']} gallons.\n\n"
                "Apply this as the live tank curve?"
            )
            footer = "OV = APPLY PROFILE   PS = RETURN"
            hint = profile_path
    elif phase == "applied":
        body = (
            f"{tank_label} Tank Calibration Applied\n\n"
            "RotorSync will reload the curve automatically."
        )
        footer = "OV = RETURN TO MENU"
        hint = calibration_state.get("profile_path") or _current_tank_calibration_profile_path(calibration_state["tank"])
    elif phase == "apply_error":
        body = (
            f"{tank_label} Tank Calibration Apply Failed\n\n"
            f"{calibration_state.get('profile_error', 'Unknown error')}"
        )
        footer = "OV = RETURN TO MENU"
        hint = _calibration_points_path()
    elif phase == "abort_confirm":
        body = (
            f"{tank_label} Tank\n\nAbort Calibration?\n\n"
            "This will discard the current calibration run."
        )
        footer = "OV = YES, ABORT   PS = NO, GO BACK"
        hint = "Use OV to exit the calibration workflow."

    # Guard refusals (dead sensor etc.) surface on the screen the operator is
    # looking at instead of silently ignoring the button press.
    if phase in ("confirm_empty", "review") and calibration_state.get("profile_error"):
        body += f"\n\n! {calibration_state['profile_error']}"

    body_font_size = 54
    if phase in ("wait_for_fill", "settling", "review", "complete", "applied", "apply_error", "abort_confirm"):
        body_font_size = 46
    elif phase in ("set_total", "set_target"):
        body_font_size = 58

    calibration_title_label.config(text=title)
    calibration_body_label.config(text=body, font=("Helvetica", body_font_size, "bold"))
    calibration_footer_label.config(text=footer)
    calibration_hint_label.config(text=hint)


def _calibration_prepare_next_step():
    global requested_gallons, fill_requested_gallons, colors_are_green
    step = calibration_state["current_step"]
    calibration_state["phase"] = "wait_for_fill"
    calibration_state["flow_started"] = False
    calibration_state["settle_deadline"] = None
    calibration_state["reading"] = None
    requested_gallons = float(step)
    fill_requested_gallons = requested_gallons
    colors_are_green = False
    draw_requested_number(f"{requested_gallons:.0f}", "red")
    _refresh_calibration_window()


def _close_calibration_window(return_to_menu=True):
    global calibration_mode, calibration_window, calibration_state
    global calibration_title_label, calibration_body_label, calibration_footer_label, calibration_hint_label
    calibration_mode = False
    calibration_state = None
    if calibration_window:
        calibration_window.destroy()
        calibration_window = None
    calibration_title_label = None
    calibration_body_label = None
    calibration_footer_label = None
    calibration_hint_label = None
    if return_to_menu:
        show_menu()


def show_tank_calibration(initial_state=None):
    global calibration_mode, calibration_window, calibration_state
    global calibration_title_label, calibration_body_label, calibration_footer_label, calibration_hint_label

    if menu_window:
        close_menu()
    if calibration_window:
        try:
            calibration_window.destroy()
        except Exception:
            pass

    calibration_mode = True
    calibration_state = initial_state or _default_calibration_state()
    calibration_window = tk.Toplevel()
    calibration_window.title("Tank Calibration")
    calibration_window.attributes('-fullscreen', True)
    calibration_window.configure(bg='black')

    calibration_title_label = tk.Label(
        calibration_window, text="", font=("Helvetica", 40, "bold"), fg="cyan", bg="black"
    )
    calibration_title_label.pack(pady=(10, 2))

    calibration_body_label = tk.Label(
        calibration_window,
        text="",
        font=("Helvetica", 54, "bold"),
        fg="white",
        bg="black",
        justify=tk.CENTER,
        wraplength=760,
    )
    calibration_body_label.pack(expand=True, fill=tk.BOTH, padx=20, pady=6)

    calibration_hint_label = tk.Label(
        calibration_window,
        text="",
        font=("Helvetica", 24, "bold"),
        fg="#ffff99",
        bg="black",
        wraplength=760,
    )
    calibration_hint_label.pack(pady=(2, 4))

    calibration_footer_label = tk.Label(
        calibration_window,
        text="",
        font=("Helvetica", 26, "bold"),
        fg="#00ffff",
        bg="#0a0a0a",
        wraplength=760,
    )
    calibration_footer_label.pack(side=tk.BOTTOM, fill=tk.X, pady=(0, 8), ipady=6)

    _refresh_calibration_window()


BASE_CALIBRATION_CSV = "/opt/mopeka/calibration-points-1070gal-tank.csv"
SENSOR_DETAILS_CSV = "/opt/mopeka/mopeka-sensor-details.csv"


def _load_calibration_curve_rows(tank):
    """[(tank_level_in, gallons)] for the tank's ACTIVE curve — its per-tank
    profile when one exists, else the shared base CSV (the converter's own
    precedence)."""
    path = _current_tank_calibration_profile_path(tank)
    if not os.path.exists(path):
        path = BASE_CALIBRATION_CSV
    rows = []
    with open(path, "r", newline="") as f:
        for row in csv.DictReader(f):
            try:
                rows.append((float(row["Tank Level (in)"]), float(row["Gallons"])))
            except (KeyError, TypeError, ValueError):
                continue
    return rows


def _selected_tank_sensor_id(tank):
    """Mopeka ID (last-3-octet MAC) of the front/back sensor from mopeka_config."""
    try:
        with open(_mopeka_config_path(), "r") as f:
            cfg = json.load(f)
    except Exception:
        return ""
    key = "back_id" if tank == "back" else "front_id"
    value = str(cfg.get(key) or "").strip()
    return "" if value.startswith("-") else value


def _read_sensor_height_offset(sensor_id):
    """Current Height Offset (inches) for a sensor from the details CSV; 0.0
    when missing/unset. The CSV has preamble rows before the real header."""
    try:
        with open(SENSOR_DETAILS_CSV, "r", newline="") as f:
            raw = list(csv.reader(f))
    except OSError:
        return 0.0
    header_idx = next((i for i, r in enumerate(raw) if "Mopeka ID" in r), None)
    if header_idx is None:
        return 0.0
    header = raw[header_idx]
    id_col = header.index("Mopeka ID")
    off_col = header.index("Height Offset")
    for row in raw[header_idx + 1:]:
        if len(row) > max(id_col, off_col) and row[id_col].strip() == sensor_id:
            try:
                return float(row[off_col])
            except (TypeError, ValueError):
                return 0.0
    return 0.0


def _write_sensor_height_offset(sensor_id, new_offset):
    """Persist a sensor's Height Offset in the details CSV, preserving the
    file's preamble rows and all other content. Atomic replace. Raises on a
    missing sensor so the wizard surfaces the failure instead of silently
    calibrating nothing."""
    with open(SENSOR_DETAILS_CSV, "r", newline="") as f:
        raw = list(csv.reader(f))
    header_idx = next((i for i, r in enumerate(raw) if "Mopeka ID" in r), None)
    if header_idx is None:
        raise ValueError("sensor CSV header not found")
    header = raw[header_idx]
    id_col = header.index("Mopeka ID")
    off_col = header.index("Height Offset")
    updated = False
    for row in raw[header_idx + 1:]:
        if len(row) > max(id_col, off_col) and row[id_col].strip() == sensor_id:
            row[off_col] = f"{new_offset:.2f}"
            updated = True
    if not updated:
        raise ValueError(f"sensor {sensor_id!r} not found in sensor CSV")
    tmp = SENSOR_DETAILS_CSV + ".tmp"
    with open(tmp, "w", newline="") as f:
        csv.writer(f).writerows(raw)
    os.replace(tmp, SENSOR_DETAILS_CSV)


def start_tank_calibration_remote(params):
    """Start the calibration wizard from a remote client (the iOS app).

    Validates everything on the caller's thread, then hands the Tk window +
    state swap to the main loop. Returns (ok, error). The remote run skips the
    interactive choose_tank/set_total/set_target phases and lands directly on
    confirm_empty with precomputed fill targets; from there the app drives the
    SAME state machine the touchscreen uses (CAL_CONFIRM/CAL_ADJUST/CAL_CANCEL).
    """
    if calibration_mode:
        return False, "calibration already running"
    if last_flow_rate >= config.FLOW_STOPPED_THRESHOLD:
        return False, "flow is active - stop the pump first"

    mode = str(params.get("mode") or "full").strip().lower()
    tank = "back" if str(params.get("tank") or "front").strip().lower() == "back" else "front"
    try:
        total_capacity = float(params.get("total_capacity") or 0)
        points = int(params.get("points") or 0)
        max_gallons = float(params.get("max_gallons") or 0)
        targets = compute_point_targets(
            mode, total_capacity=total_capacity, points=points, max_gallons=max_gallons
        )
    except (TypeError, ValueError) as exc:
        return False, str(exc)

    state = _default_calibration_state()
    state.update({
        "phase": "confirm_empty",
        "tank": tank,
        "mode": mode,
        "remote": True,
        "point_targets": targets,
        "target_index": 0,
        "max_gallons": max_gallons,
        "total_capacity": total_capacity if mode == "full" else state["total_capacity"],
        # Legacy fields kept coherent for the run archive.
        "target_capacity": targets[-1] if targets else 0,
        "step_size": targets[0] if targets else 0,
    })

    if mode == "offset":
        if not _selected_tank_sensor_id(tank):
            return False, f"no {tank} sensor configured on this box"
        curve = _load_calibration_curve_rows(tank)
        if len(curve) < 2:
            return False, "no usable calibration curve to offset against"
        state["curve_rows"] = curve

    def _begin():
        # A wizard fill must start from a zeroed totalizer: step targets are
        # cumulative totals and a stale total would auto-stop step 1 instantly.
        force_flow_reset("remote_calibration_start")
        show_tank_calibration(initial_state=state)

    root.after(0, _begin)
    return True, ""


def calibration_adjust_value(delta):
    if not calibration_state:
        return
    phase = calibration_state["phase"]
    if phase == "choose_tank":
        calibration_state["tank"] = "back" if calibration_state["tank"] == "front" else "front"
    elif phase == "set_total":
        calibration_state["total_capacity"] = max(10, calibration_state["total_capacity"] + delta)
        if calibration_state["target_capacity"] > calibration_state["total_capacity"]:
            calibration_state["target_capacity"] = calibration_state["total_capacity"]
    elif phase == "set_target":
        calibration_state["target_capacity"] = min(
            calibration_state["total_capacity"],
            max(10, calibration_state["target_capacity"] + delta),
        )
    elif phase == "review" and delta == -1:
        calibration_state["profile_error"] = ""
        calibration_state["phase"] = "settling"
        calibration_state["settle_deadline"] = time.time() + 120
    _refresh_calibration_window()


def calibration_confirm():
    if not calibration_state:
        return

    phase = calibration_state["phase"]
    if phase == "choose_tank":
        calibration_state["phase"] = "set_total"
    elif phase == "set_total":
        calibration_state["phase"] = "set_target"
    elif phase == "set_target":
        calibration_state["phase"] = "confirm_empty"
    elif phase == "confirm_empty":
        reading = _selected_tank_reading()
        # Offset mode compares readings to the curve, so a dead sensor (no
        # advertisement since boot: 0 mm AND quality 0) poisons every point —
        # refuse to start rather than "calibrate" against zeros.
        if (
            calibration_state.get("mode") == "offset"
            and (reading.get("level_mm") or 0) <= 0
            and (reading.get("quality") or 0) <= 0
        ):
            calibration_state["profile_error"] = (
                "No signal from the tank sensor — check the Mopeka, then tap begin again."
            )
            _refresh_calibration_window()
            return
        calibration_state["profile_error"] = ""
        calibration_state["points"] = []
        calibration_state["offset_diffs"] = []
        calibration_state["offset_points"] = []
        calibration_state["reading"] = reading
        if calibration_state.get("mode") == "offset":
            _record_offset_point(0.0)
        else:
            _append_calibration_point(0.0, 0)
        calibration_state["reading"] = None
        targets = calibration_state.get("point_targets")
        if targets:
            calibration_state["target_index"] = 0
            calibration_state["current_step"] = targets[0]
        else:
            calibration_state["current_step"] = calibration_state["step_size"]
        switch_mode("fill")
        _calibration_prepare_next_step()
    elif phase == "review":
        # There are gallons in the tank at every fill point, so a 0 mm reading
        # can only mean the sensor isn't reading — refuse to save the point.
        reading = calibration_state.get("reading") or {}
        if (reading.get("level_mm") or 0) <= 0:
            calibration_state["profile_error"] = (
                "Sensor reads 0 — no Mopeka signal. Fix the sensor, then use "
                "'Wait 2 more minutes' to take a fresh reading."
            )
            _refresh_calibration_window()
            return
        calibration_state["profile_error"] = ""
        if calibration_state.get("mode") == "offset":
            _record_offset_point(calibration_state["last_step_actual"])
        else:
            _append_calibration_point(
                calibration_state["last_step_actual"],
                calibration_state["current_step"],
            )
        targets = calibration_state.get("point_targets")
        if targets:
            next_index = calibration_state.get("target_index", 0) + 1
            if next_index < len(targets):
                calibration_state["target_index"] = next_index
                calibration_state["current_step"] = targets[next_index]
                _calibration_prepare_next_step()
                return
        else:
            next_step = calibration_state["current_step"] + calibration_state["step_size"]
            if next_step <= calibration_state["target_capacity"]:
                calibration_state["current_step"] = next_step
                _calibration_prepare_next_step()
                return
        _save_calibration_run()
        if calibration_state.get("mode") == "offset":
            # Preview so the operator sees the proposed offset BEFORE
            # confirming; nothing is written until the complete-phase confirm.
            try:
                calibration_state["offset_result"] = _compute_offset_result()
            except Exception:
                calibration_state["offset_result"] = None  # apply will surface the error
        calibration_state["phase"] = "complete"
    elif phase == "complete":
        try:
            if calibration_state.get("mode") == "offset":
                calibration_state["profile_path"] = _apply_offset_calibration()
            else:
                calibration_state["profile_path"] = _apply_tank_calibration_profile()
            calibration_state["profile_error"] = ""
            calibration_state["phase"] = "applied"
        except Exception as exc:
            calibration_state["profile_error"] = str(exc)
            calibration_state["phase"] = "apply_error"
    elif phase == "abort_confirm":
        _close_calibration_window(return_to_menu=True)
        return
    elif phase in ("applied", "apply_error"):
        _close_calibration_window(return_to_menu=True)
        return

    _refresh_calibration_window()


def calibration_cancel():
    if not calibration_state:
        return

    phase = calibration_state["phase"]
    if phase == "choose_tank":
        calibration_state["return_phase"] = "choose_tank"
        calibration_state["phase"] = "abort_confirm"
        return
    if phase in ("complete", "applied", "apply_error"):
        _close_calibration_window(return_to_menu=True)
        return
    if phase == "set_total":
        calibration_state["phase"] = "choose_tank"
    elif phase == "set_target":
        calibration_state["phase"] = "set_total"
    elif phase == "confirm_empty":
        if calibration_state.get("remote"):
            calibration_state["return_phase"] = "confirm_empty"
            calibration_state["phase"] = "abort_confirm"
        else:
            calibration_state["phase"] = "set_target"
    elif phase == "abort_confirm":
        calibration_state["phase"] = calibration_state.get("return_phase") or "choose_tank"
        calibration_state["return_phase"] = None
    else:
        calibration_state["return_phase"] = phase
        calibration_state["phase"] = "abort_confirm"
    _refresh_calibration_window()


def calibration_reread_now():
    if calibration_state and calibration_state["phase"] == "review":
        calibration_state["reading"] = _selected_tank_reading()
        _refresh_calibration_window()

def run_self_test():
    """Run system self-test"""
    global self_test_mode, self_test_window

    self_test_mode = True
    self_test_window = tk.Toplevel()
    self_test_window.title("System Self-Test")
    self_test_window.attributes('-fullscreen', True)
    self_test_window.configure(bg='black')

    # Title
    title = tk.Label(self_test_window, text="SYSTEM SELF-TEST", font=("Helvetica", 44, "bold"),
                     fg="cyan", bg="black")
    title.pack(pady=(12, 6))

    # Controls instruction
    controls = tk.Label(self_test_window, text="OV=EXIT TO MENU",
                       font=("Helvetica", 28, "bold"), fg="#ffff00", bg="#0a0a0a")
    controls.pack(pady=2)

    # Results frame
    results_frame = tk.Frame(self_test_window, bg='black')
    results_frame.pack(fill=tk.BOTH, expand=True, padx=24, pady=(8, 14))

    results_text = tk.Text(results_frame, font=("Courier", 32, "bold"), bg="black", fg="white",
                           wrap=tk.WORD, borderwidth=0, highlightthickness=0)
    results_text.pack(fill=tk.BOTH, expand=True)

    def run_tests():
        results_text.insert(tk.END, "Starting self-test...\n\n")
        results_text.update()

        # Test 1: GPIO
        results_text.insert(tk.END, "1. GPIO Test: ")
        results_text.update()
        if GPIO_AVAILABLE:
            try:
                # Quick relay pulse test (0.5 seconds)
                GPIO.output(config.PUMP_STOP_RELAY_PIN, GPIO.HIGH)
                time.sleep(0.5)
                GPIO.output(config.PUMP_STOP_RELAY_PIN, GPIO.LOW)
                results_text.insert(tk.END, "PASS (Relay pulsed)\n", "pass")
            except Exception as e:
                results_text.insert(tk.END, f"FAIL ({e})\n", "fail")
        else:
            results_text.insert(tk.END, "SKIP (Not available)\n", "skip")
        results_text.update()

        # Test 2: Serial Port
        results_text.insert(tk.END, "2. Serial Port Test: ")
        results_text.update()
        try:
            import serial
            ser = serial.Serial(config.SERIAL_PORT, config.SERIAL_BAUD, timeout=1)
            ser.close()
            results_text.insert(tk.END, "PASS\n", "pass")
        except Exception as e:
            results_text.insert(tk.END, f"FAIL ({e})\n", "fail")
        results_text.update()

        # Test 3: IOL-HAT
        results_text.insert(tk.END, "3. IOL-HAT Communication: ")
        results_text.update()
        try:
            with iol_io_lock:
                raw_data = iolhat.pd(config.IOL_PORT, 0, config.DATA_LENGTH, None)
            if len(raw_data) >= 15 and raw_data != b'\x00' * len(raw_data):
                results_text.insert(tk.END, "PASS (Data received)\n", "pass")
            else:
                results_text.insert(tk.END, "FAIL (No response or all zeros)\n", "fail")
        except Exception as e:
            results_text.insert(tk.END, f"FAIL ({e})\n", "fail")
        results_text.update()

        # Test 4: Flow Meter
        results_text.insert(tk.END, "4. Flow Meter Reading: ")
        results_text.update()
        try:
            gallons = get_cached_actual_gallons() if flow_control_active() else read_flow_meter()
            if not connection_error:
                results_text.insert(tk.END, f"PASS (Reading: {gallons:.2f} gal)\n", "pass")
            else:
                results_text.insert(tk.END, f"FAIL ({error_message})\n", "fail")
        except Exception as e:
            results_text.insert(tk.END, f"FAIL ({e})\n", "fail")
        results_text.update()

        # Test 5: Display
        results_text.insert(tk.END, "5. Display System: PASS (You can see this)\n", "pass")

        results_text.insert(tk.END, "\n=== Test Complete ===\n")

        # Color tags
        results_text.tag_config("pass", foreground="green")
        results_text.tag_config("fail", foreground="red")
        results_text.tag_config("skip", foreground="yellow")

    # Run tests in thread to not block GUI
    test_thread = threading.Thread(target=run_tests, daemon=True)
    test_thread.start()

def close_self_test():
    """Close self-test and return to menu"""
    global self_test_mode, self_test_window
    self_test_mode = False
    if self_test_window:
        self_test_window.destroy()
        self_test_window = None
    arm_menu_ov_guard()

def run_full_test():
    """Run comprehensive interactive full system test"""
    global full_test_mode, full_test_window

    full_test_mode = True
    full_test_window = tk.Toplevel()
    full_test_window.title("Full System Test")
    full_test_window.attributes('-fullscreen', True)
    full_test_window.configure(bg='black')

    # Title
    title = tk.Label(full_test_window, text="FULL SYSTEM TEST", font=("Helvetica", 36, "bold"),
                     fg="cyan", bg="black")
    title.pack(pady=15)

    # Controls instruction
    controls = tk.Label(full_test_window, text="Test each item - OV=TEST OV / EXIT MENU (press twice to exit)",
                       font=("Helvetica", 18, "bold"), fg="#ffff00", bg="#0a0a0a")
    controls.pack(pady=2)

    # Test status dict
    test_status = {
        'minus_1': False,
        'plus_1': False,
        'minus_10': False,
        'plus_10': False,
        'OV': False,
        'PS': False,
        'button': False,
        'gpio': False,
        'relay': False,
        'serial': False,
        'iolhat': False,
        'flow_meter': False,
        'display': False
    }

    # Flow meter error message
    flow_meter_error = [None]  # Use list to allow modification in nested function

    # Results frame
    results_frame = tk.Frame(full_test_window, bg='black')
    results_frame.pack(fill=tk.BOTH, expand=True, padx=30, pady=10)

    results_text = tk.Text(results_frame, font=("Courier", 20), bg="black", fg="white",
                           height=18, width=65)
    results_text.pack()

    # Tag configs for colors
    results_text.tag_config("pass", foreground="green")
    results_text.tag_config("pending", foreground="yellow")
    results_text.tag_config("fail", foreground="red")
    results_text.tag_config("header", foreground="cyan")

    def update_display():
        results_text.delete('1.0', tk.END)
        results_text.insert(tk.END, "SERIAL COMMANDS:\n", "header")
        results_text.insert(tk.END, f"  Send -1:     {'✓ PASS' if test_status['minus_1'] else '○ PENDING'}\n",
                           "pass" if test_status['minus_1'] else "pending")
        results_text.insert(tk.END, f"  Send +1:     {'✓ PASS' if test_status['plus_1'] else '○ PENDING'}\n",
                           "pass" if test_status['plus_1'] else "pending")
        results_text.insert(tk.END, f"  Send -10:    {'✓ PASS' if test_status['minus_10'] else '○ PENDING'}\n",
                           "pass" if test_status['minus_10'] else "pending")
        results_text.insert(tk.END, f"  Send +10:    {'✓ PASS' if test_status['plus_10'] else '○ PENDING'}\n",
                           "pass" if test_status['plus_10'] else "pending")
        results_text.insert(tk.END, f"  Send OV:     {'✓ PASS' if test_status['OV'] else '○ PENDING'}\n",
                           "pass" if test_status['OV'] else "pending")
        results_text.insert(tk.END, f"  Send PS:     {'✓ PASS' if test_status['PS'] else '○ PENDING'}\n\n",
                           "pass" if test_status['PS'] else "pending")

        results_text.insert(tk.END, "HARDWARE:\n", "header")
        results_text.insert(tk.END, f"  GPIO:        {'✓ PASS' if test_status['gpio'] else '○ TESTING...'}\n",
                           "pass" if test_status['gpio'] else "pending")
        results_text.insert(tk.END, f"  Relay:       {'✓ PASS' if test_status['relay'] else '○ TESTING...'}\n",
                           "pass" if test_status['relay'] else "pending")
        results_text.insert(tk.END, f"  Serial Port: {'✓ PASS' if test_status['serial'] else '○ TESTING...'}\n",
                           "pass" if test_status['serial'] else "pending")
        results_text.insert(tk.END, f"  IOL-HAT:     {'✓ PASS' if test_status['iolhat'] else '○ TESTING...'}\n",
                           "pass" if test_status['iolhat'] else "pending")

        # Flow meter with error display
        if flow_meter_error[0]:
            results_text.insert(tk.END, f"  Flow Meter:  ✗ ERROR\n", "fail")
            results_text.insert(tk.END, f"\n{flow_meter_error[0]}\n", "fail")
        else:
            results_text.insert(tk.END, f"  Flow Meter:  {'✓ PASS' if test_status['flow_meter'] else '○ TESTING...'}\n",
                               "pass" if test_status['flow_meter'] else "pending")

        results_text.insert(tk.END, f"  Button:      {'✓ PASS' if test_status['button'] else '○ PENDING (Press thumbs up)'}\n",
                           "pass" if test_status['button'] else "pending")
        results_text.insert(tk.END, f"  Display:     {'✓ PASS' if test_status['display'] else '○ PASS (You can see this)'}\n",
                           "pass")

    def mark_tested(test_name):
        test_status[test_name] = True
        update_display()

    # Store functions in window for serial listener access
    full_test_window.mark_tested = mark_tested
    full_test_window.test_status = test_status

    # Run hardware tests automatically
    def run_hardware_tests():
        # Test 1: GPIO
        if GPIO_AVAILABLE:
            try:
                # Quick test - just check if GPIO is initialized
                root.after(0, lambda: mark_tested('gpio'))
            except Exception:
                pass

        time.sleep(0.2)

        # Test 2: Relay (with visible pulse)
        if GPIO_AVAILABLE:
            try:
                GPIO.output(config.PUMP_STOP_RELAY_PIN, GPIO.HIGH)
                time.sleep(0.3)
                GPIO.output(config.PUMP_STOP_RELAY_PIN, GPIO.LOW)
                root.after(0, lambda: mark_tested('relay'))
            except Exception:
                pass

        time.sleep(0.2)

        # Test 3: Serial Port
        try:
            import serial
            ser = serial.Serial(config.SERIAL_PORT, config.SERIAL_BAUD, timeout=1)
            ser.close()
            root.after(0, lambda: mark_tested('serial'))
        except Exception:
            pass

        time.sleep(0.2)

        # Test 4: IOL-HAT Communication (with 5 second timeout)
        iolhat_error = [None]
        iolhat_success = [False]

        def test_iolhat():
            try:
                with iol_io_lock:
                    raw_data = iolhat.pd(config.IOL_PORT, 0, config.DATA_LENGTH, None)
                if len(raw_data) >= 15 and raw_data != b'\x00' * len(raw_data):
                    iolhat_success[0] = True
                    root.after(0, lambda: mark_tested('iolhat'))
                # If all zeros, it means IOL-HAT works but device isn't responding
                elif len(raw_data) >= 15:
                    iolhat_success[0] = True
                    root.after(0, lambda: mark_tested('iolhat'))
            except Exception as e:
                iolhat_error[0] = str(e)

        # Run IOL-HAT test with timeout
        iolhat_thread = threading.Thread(target=test_iolhat, daemon=True)
        iolhat_thread.start()
        iolhat_thread.join(timeout=5.0)  # 5 second timeout

        # Check if test completed or timed out
        if not iolhat_success[0]:
            # Test failed or timed out
            pass  # IOL-HAT test remains in TESTING state (will show as yellow/pending)

        time.sleep(0.2)

        # Test 5: Flow Meter
        try:
            with iol_io_lock:
                raw_data = iolhat.pd(config.IOL_PORT, 0, config.DATA_LENGTH, None)
            if raw_data and len(raw_data) == config.DATA_LENGTH and raw_data != b'\x00' * len(raw_data):
                root.after(0, lambda: mark_tested('flow_meter'))
            else:
                # Flow meter returned all zeros or wrong length
                error_msg = "ERROR: Ensure flow meter is connected.\nCheck if flow meter displays green check mark in corner."
                flow_meter_error[0] = error_msg
                root.after(0, update_display)
        except Exception as e:
            error_msg = f"ERROR: Ensure flow meter is connected.\nCheck if flow meter displays green check mark in corner.\n({str(e)})"
            flow_meter_error[0] = error_msg
            root.after(0, update_display)

        # Test 6: Display (automatically pass - if you can see this)
        root.after(0, lambda: mark_tested('display'))

    update_display()

    # Start hardware tests in background
    hardware_thread = threading.Thread(target=run_hardware_tests, daemon=True)
    hardware_thread.start()

def close_full_test():
    """Close full-test and return to menu"""
    global full_test_mode, full_test_window
    full_test_mode = False
    if full_test_window:
        full_test_window.destroy()
        full_test_window = None
    arm_menu_ov_guard()

def check_wifi_status():
    """Check if WiFi is connected and return status string"""
    try:
        import subprocess
        # Check if wlan0 is up and has an IP
        result = subprocess.run(['ip', 'addr', 'show', 'wlan0'],
                              capture_output=True, text=True, timeout=1)

        if 'state UP' in result.stdout and 'inet ' in result.stdout:
            # Get signal strength if available
            try:
                signal_result = subprocess.run(['cat', '/proc/net/wireless'],
                                             capture_output=True, text=True, timeout=1)
                if 'wlan0' in signal_result.stdout:
                    return "WiFi: CONNECTED"
            except Exception:
                pass
            return "WiFi: CONNECTED"
        else:
            return "WiFi: DISCONNECTED"
    except Exception as e:
        return "WiFi: UNKNOWN"

def run_system_update():
    """Run system update"""
    global update_mode, update_window

    update_mode = True
    update_window = tk.Toplevel()
    update_window.title("System Update")
    update_window.attributes('-fullscreen', True)
    update_window.configure(bg='black')

    # Title
    title = tk.Label(update_window, text="SYSTEM UPDATE", font=("Helvetica", 36, "bold"),
                     fg="cyan", bg="black")
    title.pack(pady=20)

    # Controls instruction
    controls = tk.Label(update_window, text="OV=EXIT TO MENU",
                       font=("Helvetica", 22, "bold"), fg="#ffff00", bg="#0a0a0a")
    controls.pack(pady=2)

    # Status text
    status_frame = tk.Frame(update_window, bg='black')
    status_frame.pack(fill=tk.BOTH, expand=True, padx=40, pady=20)

    status_text = tk.Text(status_frame, font=("Courier", 32, "bold"), bg="black", fg="lime",
                         wrap=tk.WORD)
    status_text.pack(fill=tk.BOTH, expand=True)

    def run_update():
        import subprocess

        def _deployed_runtime_stale():
            # True when the RUNNING code in /opt has drifted from the repo — e.g.
            # the git tree was advanced (SD-card clone, or a manual `git pull`)
            # without the deploy step ever copying repo -> /opt. "Up to date" is
            # judged from git commits alone, so such a box would otherwise report
            # current forever and never repair its stale runtime (this is exactly
            # how a cloned box ran old BLE code with a hard-pinned adapter MAC).
            repo = '/home/pi/Big-Beautiful-Box'

            def _same(a, b):
                try:
                    with open(a, 'rb') as fa, open(b, 'rb') as fb:
                        return fa.read() == fb.read()
                except OSError:
                    return False

            for name in ('rotorsync_bumble.py', 'rotorsync_watchdog.py'):
                src = os.path.join(repo, name)
                if os.path.exists(src) and not _same(src, os.path.join('/opt', name)):
                    return True
            src_dir = os.path.join(repo, 'src')
            if os.path.isdir(src_dir):
                for root, _dirs, files in os.walk(src_dir):
                    for fn in files:
                        if not fn.endswith('.py'):
                            continue
                        s = os.path.join(root, fn)
                        rel = os.path.relpath(s, src_dir)
                        if not _same(s, os.path.join('/opt/src', rel)):
                            return True
            return False

        status_text.insert(tk.END, "Starting BBB software update from GitHub...\n\n")
        status_text.update()

        try:
            # Show current version
            status_text.insert(tk.END, f"Current Version: {VERSION}\n\n")
            status_text.update()

            if not os.path.isdir('/home/pi/Big-Beautiful-Box/.git'):
                status_text.insert(tk.END, "ERROR: Installed BBB folder is not a git repository.\n")
                status_text.insert(tk.END, "This box must be installed from a real git checkout for updates to work.\n")
                status_text.insert(tk.END, "Re-run install.sh from a cloned BBB repo.\n")
                status_text.insert(tk.END, "Press OV to return to menu\n")
                status_text.update()
                return

            # Step 1: Navigate to git repo and pull latest
            status_text.insert(tk.END, "=== Step 1: Checking for updates ===\n")
            status_text.update()

            result = subprocess.run(['git', '-C', '/home/pi/Big-Beautiful-Box', 'fetch', 'origin'],
                                  capture_output=True, text=True, timeout=60)
            if result.stderr:
                status_text.insert(tk.END, result.stderr + "\n")
            status_text.update()

            if result.returncode != 0:
                status_text.insert(tk.END, "ERROR: Git fetch failed!\n")
                status_text.insert(tk.END, "Press OV to return to menu\n")
                return

            # Check whether anything new exists on origin/master
            result = subprocess.run(
                ['git', '-C', '/home/pi/Big-Beautiful-Box', 'rev-list', '--count', 'HEAD..origin/master'],
                capture_output=True, text=True, timeout=10
            )
            updates_available = int(result.stdout.strip() or "0")
            # Self-heal a stale runtime even when git is already current — a
            # cloned/hand-pulled box can be "up to date" on commits yet still run
            # old code in /opt because the deploy step never ran.
            runtime_stale = _deployed_runtime_stale()

            # Get the version we're updating to
            result = subprocess.run(['git', '-C', '/home/pi/Big-Beautiful-Box', 'log', 'origin/master', '-1', '--format=%s'],
                                  capture_output=True, text=True, timeout=10)
            new_version_msg = result.stdout.strip()
            new_version = _read_git_ref_version('origin/master') or "(version file missing)"
            if updates_available == 0 and not runtime_stale:
                status_text.insert(tk.END, "\nAlready up to date.\n")
                status_text.insert(tk.END, f"Latest Version: {new_version}\n")
                status_text.insert(tk.END, f"Latest commit:\n{new_version_msg}\n\n")
                status_text.insert(tk.END, "Press OV to return to menu\n")
                status_text.update()
                return
            if updates_available == 0 and runtime_stale:
                status_text.insert(tk.END, "\nSoftware is current, but the deployed runtime is stale.\n")
                status_text.insert(tk.END, "Repairing the running code (re-deploying to /opt)...\n\n")
                status_text.update()

            status_text.insert(tk.END, f"\nNew Version Available: {new_version}\n")
            status_text.insert(tk.END, f"Commit:\n{new_version_msg}\n")
            status_text.insert(tk.END, f"Commits to apply: {updates_available}\n\n")
            status_text.update()

            # Remember the currently-running (known-good) commit so we can roll the
            # working tree back if the incoming code fails to compile — otherwise a
            # bad push would crash-loop the dashboard with no recovery in the field.
            old_commit = subprocess.run(
                ['git', '-C', '/home/pi/Big-Beautiful-Box', 'rev-parse', 'HEAD'],
                capture_output=True, text=True, timeout=10
            ).stdout.strip()

            # Only touch the git tree when there are real commits to apply. In the
            # runtime-repair path (git already current, /opt stale) the tree is
            # correct — skip the reset and go straight to compile-gate + deploy.
            if updates_available > 0:
                # Step 2: Reset to latest origin/master
                status_text.insert(tk.END, "=== Step 2: Installing update ===\n")
                status_text.update()

                result = subprocess.run(['git', '-C', '/home/pi/Big-Beautiful-Box', 'reset', '--hard', 'origin/master'],
                                      capture_output=True, text=True, timeout=30)
                status_text.insert(tk.END, "Updated repository\n")
                status_text.update()

                if result.returncode != 0:
                    status_text.insert(tk.END, "ERROR: Git reset failed!\n")
                    status_text.insert(tk.END, "Press OV to return to menu\n")
                    return

            # Compile-gate the freshly-checked-out code BEFORE deploying/restarting
            # (mirrors the BLE update path). If it won't even import, roll the tree
            # back to the last known-good commit and abort WITHOUT restarting, so a
            # broken push can't brick the box in the field.
            repo = '/home/pi/Big-Beautiful-Box'
            compile_main = subprocess.run(
                ['python3', '-m', 'py_compile', repo + '/dashboard.py', repo + '/rotorsync_bumble.py'],
                capture_output=True, text=True, timeout=60
            )
            compile_src = subprocess.run(
                ['python3', '-m', 'compileall', '-q', repo + '/src'],
                capture_output=True, text=True, timeout=60
            )
            if compile_main.returncode != 0 or compile_src.returncode != 0:
                if old_commit:
                    subprocess.run(['git', '-C', repo, 'reset', '--hard', old_commit],
                                   capture_output=True, text=True, timeout=30)
                status_text.insert(tk.END, "ERROR: update failed to compile — rolled back to the running version.\n")
                status_text.insert(tk.END, "NOT restarting; the box keeps running the previous code.\n")
                err = (compile_main.stderr or compile_src.stderr or "").strip()
                if err:
                    status_text.insert(tk.END, err[:500] + "\n")
                status_text.insert(tk.END, "Press OV to return to menu\n")
                status_text.update()
                return

            status_text.insert(tk.END, "Refreshing deployed BBB runtime...\n")
            status_text.update()

            deploy_cmd = (
                "set -e; "
                "cp /home/pi/Big-Beautiful-Box/deploy/bbb-logrotate.conf /etc/logrotate.d/bbb; "
                "cp /home/pi/Big-Beautiful-Box/deploy/bbb-logrotate.service /etc/systemd/system/bbb-logrotate.service; "
                "cp /home/pi/Big-Beautiful-Box/deploy/bbb-logrotate.timer /etc/systemd/system/bbb-logrotate.timer; "
                "mkdir -p /opt/src /opt/mopeka; "
                "cp /home/pi/Big-Beautiful-Box/rotorsync_bumble.py /opt/rotorsync_bumble.py; "
                "cp /home/pi/Big-Beautiful-Box/rotorsync_watchdog.py /opt/rotorsync_watchdog.py; "
                "cp -r /home/pi/Big-Beautiful-Box/src/. /opt/src/; "
                "for mopeka_file in /home/pi/Big-Beautiful-Box/mopeka/*; do "
                "[ -f \"$mopeka_file\" ] || continue; "
                "base_name=$(basename \"$mopeka_file\"); "
                "if { [ \"$base_name\" = \"mopeka_config.json\" ] || [ \"$base_name\" = \"mopeka-sensor-details.csv\" ]; } "
                "&& [ -f \"/opt/mopeka/$base_name\" ]; then continue; fi; "
                "cp \"$mopeka_file\" /opt/mopeka/; "
                "done; "
                "chown -R pi:pi /opt/mopeka; "
                # Purge a RETIRED maintenance secret so the box re-adopts the
                # current shared fleet key on its next maintenance session. A box
                # that adopted a now-retired key (e.g. a company's interim key
                # before all servers converged on the shared key) is otherwise
                # locked out forever - it can't verify the new key's frames to
                # receive a replacement, and the persisted secret is first-wins.
                # We ship only sha256 fingerprints of retired keys (repo is
                # public); a match means delete-and-re-adopt. Mirrors the
                # runtime read (bytes.strip()) exactly. Fail-soft.
                "python3 - <<'PYRETIRE'\n"
                "import hashlib, pathlib\n"
                "retired=set()\n"
                "rp=pathlib.Path('/home/pi/Big-Beautiful-Box/deploy/retired-maintenance-secrets.txt')\n"
                "try:\n"
                "    if rp.exists():\n"
                "        for line in rp.read_text().splitlines():\n"
                "            line=line.strip()\n"
                "            if line and not line.startswith('#'):\n"
                "                retired.add(line.lower())\n"
                "except OSError:\n"
                "    retired=set()\n"
                "for p in (pathlib.Path('/etc/rotorsync/maintenance.secret'), pathlib.Path('/home/pi/.rotorsync-maintenance-secret')):\n"
                "    try:\n"
                "        if not p.exists():\n"
                "            continue\n"
                "        fp=hashlib.sha256(p.read_bytes().strip()).hexdigest().lower()\n"
                "        if fp in retired:\n"
                "            p.unlink()\n"
                "            print('[update] retired maintenance secret purged from %s; box will re-adopt the current fleet key on next session' % p)\n"
                "    except OSError as e:\n"
                "        print('[update] could not check %s: %s' % (p, e))\n"
                "PYRETIRE\n"
                "if [ ! -s /etc/rotorsync/maintenance.secret ] && [ ! -s /home/pi/.rotorsync-maintenance-secret ]; then "
                "if [ -n \"${BBB_MAINTENANCE_SECRET:-${MAINTENANCE_RELAY_SECRET:-}}\" ]; then "
                "umask 077; printf '%s' \"${BBB_MAINTENANCE_SECRET:-${MAINTENANCE_RELAY_SECRET:-}}\" > /home/pi/.rotorsync-maintenance-secret; chown pi:pi /home/pi/.rotorsync-maintenance-secret; chmod 600 /home/pi/.rotorsync-maintenance-secret; "
                "elif [ -s /home/pi/Big-Beautiful-Box/deploy/maintenance.secret.local ]; then "
                "install -m 600 -o pi -g pi /home/pi/Big-Beautiful-Box/deploy/maintenance.secret.local /home/pi/.rotorsync-maintenance-secret; "
                "fi; "
                "fi; "
                "if [ -x /home/pi/Big-Beautiful-Box/deploy/setup-cursor-control.sh ]; then "
                "/home/pi/Big-Beautiful-Box/deploy/setup-cursor-control.sh --restart-dashboard; "
                "fi; "
                # Quiet the Ubuntu update/release nags fleet-wide (held at one OS
                # level, upgraded out of season — see AGENTS.md). Fail-soft.
                "if [ -x /home/pi/Big-Beautiful-Box/deploy/quiet-os-update-nag.sh ]; then "
                "/home/pi/Big-Beautiful-Box/deploy/quiet-os-update-nag.sh || true; "
                "fi; "
                "chmod 755 /opt/rotorsync_bumble.py /opt/rotorsync_watchdog.py; "
                "python3 - <<'PY'\n"
                "from pathlib import Path\n"
                "cfg = Path('/boot/firmware/config.txt')\n"
                "text = cfg.read_text()\n"
                "text = text.replace('hdmi_group=2', 'hdmi_group=1')\n"
                "text = text.replace('hdmi_mode=87', 'hdmi_mode=34')\n"
                "text = text.replace('hdmi_cvt=1024 600 60 6 0 0 0\\n', '')\n"
                "for line in ('hdmi_force_hotplug=1', 'hdmi_drive=2', 'hdmi_group=1', 'hdmi_mode=34'):\n"
                "    if line not in text:\n"
                "        text += ('\\n' if not text.endswith('\\n') else '') + line + '\\n'\n"
                "cfg.write_text(text)\n"
                "video_arg = 'video=HDMI-A-1:1920x1080M@30D'\n"
                "for path_str in ('/boot/firmware/current/cmdline.txt', '/boot/firmware/cmdline.txt', '/boot/cmdline.txt'):\n"
                "    p = Path(path_str)\n"
                "    if not p.exists():\n"
                "        continue\n"
                "    parts = p.read_text().strip().split()\n"
                "    if video_arg not in parts:\n"
                "        parts.append(video_arg)\n"
                "        p.write_text(' '.join(parts) + '\\n')\n"
                "PY\n"
                # RotorLink (WiFi link iPad<->dashboard + WiFi maintenance terminal)
                "command -v avahi-publish-service >/dev/null 2>&1 || apt-get install -y avahi-utils >/dev/null 2>&1 || true; "
                "python3 -c 'import websockets' >/dev/null 2>&1 || apt-get install -y python3-websockets >/dev/null 2>&1 || python3 -m pip install --break-system-packages websockets >/dev/null 2>&1 || true; "
                # Ensure bumble/bleak are installed for the SYSTEM interpreter (root) that
                # runs rotorsync.service. Some boxes only had them in pi's ~/.local, so the
                # root service crash-looped "No module named 'bumble'" on restart/reboot.
                # Pinned to match install.sh so the whole fleet converges to one version.
                # Fail-soft (|| true): a transient offline run must not abort the update.
                "python3 -m pip install --break-system-packages --ignore-installed bleak bumble==0.0.229 > /tmp/bbb-bumble-install.log 2>&1 || true; "
                "mkdir -p /etc/rotorlink; [ -f /etc/rotorlink/ap.psk ] || printf 'rotorsync' > /etc/rotorlink/ap.psk; "
                "cp /home/pi/Big-Beautiful-Box/systemd/rotorlink.service /etc/systemd/system/rotorlink.service; "
                "systemctl daemon-reload; "
                "systemctl enable --now rotorlink.service || true; "
                "systemctl enable --now bbb-logrotate.timer; "
                "systemctl start bbb-logrotate.service"
            )
            result = subprocess.run(
                ['bash', '-lc', f"printf 'raspi\\n' | sudo -S bash -lc {shlex.quote(deploy_cmd)}"],
                capture_output=True, text=True, timeout=180
            )
            if result.returncode != 0:
                status_text.insert(tk.END, "WARNING: Could not refresh deployed BBB runtime.\n")
                if result.stderr:
                    status_text.insert(tk.END, result.stderr + "\n")
            else:
                status_text.insert(tk.END, "BBB runtime files updated.\n")
                status_text.insert(tk.END, "Boot display config updated to 1080p30.\n")
            status_text.update()

            status_text.insert(tk.END, "\n=== UPDATE COMPLETE ===\n\n")
            status_text.insert(tk.END, "Restarting service to apply changes...\n\n")
            status_text.update()

            # Wait 2 seconds so user can see the message
            time.sleep(2)

            # Restart via systemd so both IOL master and dashboard restart cleanly.
            # Launch in the background because the current process will be terminated by the restart.
            restart_cmd = (
                "sleep 1; "
                "printf 'raspi\n' | sudo -S systemctl restart rotorsync.service rotorsync_watchdog.service iol_dashboard.service rotorlink.service"
            )
            subprocess.Popen(['bash', '-lc', restart_cmd])
            return

        except subprocess.TimeoutExpired:
            status_text.insert(tk.END, "\n\nERROR: Command timed out\n")
            status_text.insert(tk.END, "Press OV to return to menu\n")
        except Exception as e:
            status_text.insert(tk.END, f"\n\nERROR: {e}\n")
            status_text.insert(tk.END, "Press OV to return to menu\n")

        status_text.see(tk.END)

    # Run update in thread
    update_thread = threading.Thread(target=run_update, daemon=True)
    update_thread.start()


def run_bug_capture():
    """Capture a compact bug report with the most useful recent diagnostics."""
    global update_mode, update_window

    update_mode = True
    update_window = tk.Toplevel()
    update_window.title("Capture Bug Report")
    update_window.attributes('-fullscreen', True)
    update_window.configure(bg='black')

    title = tk.Label(update_window, text="CAPTURE BUG REPORT", font=("Helvetica", 36, "bold"),
                     fg="cyan", bg="black")
    title.pack(pady=20)

    controls = tk.Label(update_window, text="OV=EXIT TO MENU",
                       font=("Helvetica", 22, "bold"), fg="#ffff00", bg="#0a0a0a")
    controls.pack(pady=2)

    status_frame = tk.Frame(update_window, bg='black')
    status_frame.pack(fill=tk.BOTH, expand=True, padx=40, pady=20)

    status_text = tk.Text(status_frame, font=("Courier", 24, "bold"), bg="black", fg="lime",
                         wrap=tk.WORD)
    status_text.pack(fill=tk.BOTH, expand=True)

    def append(msg):
        status_text.insert(tk.END, msg)
        status_text.see(tk.END)
        status_text.update()

    def run_capture():
        timestamp = time.strftime('%Y%m%d-%H%M%S')
        report_dir = "/home/pi/bug_reports"
        report_path = f"{report_dir}/bug-report-{timestamp}.txt"
        capture_script = f"""set -e
mkdir -p {report_dir}
{{
  echo "BBB Bug Report"
  echo "Generated: $(date)"
  echo "Version: {VERSION}"
  echo
  echo "=== SYSTEM ==="
  uname -a
  echo
  uptime
  echo
  df -h /
  echo
  echo "=== SERVICES ==="
  systemctl --no-pager --full status iol_dashboard.service rotorsync.service rotorsync_watchdog.service || true
  echo
  echo "=== JOURNAL: IOL DASHBOARD (LAST 120) ==="
  journalctl -u iol_dashboard.service -n 120 --no-pager || true
  echo
  echo "=== JOURNAL: ROTORSYNC (LAST 120) ==="
  journalctl -u rotorsync.service -n 120 --no-pager || true
  echo
  echo "=== FILL HISTORY (LAST 40) ==="
  tail -n 40 /home/pi/fill_history.log 2>/dev/null || true
  echo
  echo "=== FILL CALIBRATION (LAST 40) ==="
  tail -n 40 /home/pi/fill_calibration.log 2>/dev/null || true
  echo
  echo "=== SERIAL DEBUG (LAST 200) ==="
  tail -n 200 /home/pi/serial_debug.log 2>/dev/null || true
  echo
  echo "=== MENU DEBUG (LAST 120) ==="
  tail -n 120 /home/pi/menu_debug.log 2>/dev/null || true
  echo
  echo "=== BUTTON DEBUG (LAST 120) ==="
  tail -n 120 /home/pi/button_debug.log 2>/dev/null || true
  echo
  echo "=== WATCHDOG LOG (LAST 120) ==="
  tail -n 120 /home/pi/rotorsync_watchdog.log 2>/dev/null || true
  echo
  echo "=== IOL DASHBOARD LOG (LAST 400) ==="
  tail -n 400 /home/pi/iol_dashboard.log 2>/dev/null || true
}} > {report_path}
echo {report_path}
"""

        append("Capturing bug report...\n\n")
        try:
            result = subprocess.run(
                ['bash', '-lc', capture_script],
                capture_output=True,
                text=True,
                timeout=30,
            )
            if result.returncode != 0:
                append("ERROR: Bug capture failed\n\n")
                if result.stderr:
                    append(result.stderr + "\n")
            else:
                path = result.stdout.strip().splitlines()[-1]
                append("Bug report captured successfully.\n\n")
                append(f"Saved to:\n{path}\n\n")
                append("Press OV to return to menu\n")
        except subprocess.TimeoutExpired:
            append("ERROR: Bug capture timed out\n\nPress OV to return to menu\n")
        except Exception as e:
            append(f"ERROR: {e}\n\nPress OV to return to menu\n")

    capture_thread = threading.Thread(target=run_capture, daemon=True)
    capture_thread.start()

def close_update():
    """Close update window and return to menu"""
    global update_mode, update_window
    update_mode = False
    if update_window:
        update_window.destroy()
        update_window = None
    arm_menu_ov_guard()

def confirm_reset_season():
    """Confirm and reset season total with OV/PS commands"""
    # Create confirmation window
    confirm_window = tk.Toplevel()
    confirm_window.title("Reset Season Confirmation")
    confirm_window.attributes('-fullscreen', True)
    confirm_window.configure(bg='black')

    # Confirmation message
    message = tk.Label(confirm_window, text="RESET SEASON TOTAL?",
                      font=("Helvetica", 48, "bold"), fg="orange", bg="black")
    message.pack(expand=True, pady=50)

    # Show current season total
    current_total = tk.Label(confirm_window, text=f"Current Season Total: {season_total:.2f} gal",
                            font=("Helvetica", 32, "bold"), fg="lime", bg="black")
    current_total.pack(pady=20)

    # Instructions
    instructions = tk.Label(confirm_window, text="Press OV to CONFIRM or PS to CANCEL",
                          font=("Helvetica", 22, "bold"), fg="cyan", bg="black")
    instructions.pack(pady=30)

    # Countdown timer
    countdown_label = tk.Label(confirm_window, text="Auto-cancel in 10 seconds",
                             font=("Helvetica", 22), fg="white", bg="black")
    countdown_label.pack(pady=20)

    countdown = [10]  # Use list to allow modification in nested function

    def cancel_reset():
        global reset_season_confirm_window, reset_season_confirm_handler, reset_season_cancel_handler
        confirm_window.destroy()
        reset_season_confirm_window = None
        reset_season_confirm_handler = None
        reset_season_cancel_handler = None

    def confirm_reset():
        global reset_season_confirm_window, reset_season_confirm_handler, reset_season_cancel_handler
        print("Resetting season total...")
        reset_season_total()
        confirm_window.destroy()
        reset_season_confirm_window = None
        reset_season_confirm_handler = None
        reset_season_cancel_handler = None
        # Update menu display
        if menu_window:
            update_totals_display()

    def update_countdown():
        countdown[0] -= 1
        if countdown[0] <= 0:
            cancel_reset()
        else:
            countdown_label.config(text=f"Auto-cancel in {countdown[0]} seconds")
            confirm_window.after(1000, update_countdown)

    # Bind keyboard shortcuts
    confirm_window.bind('<Escape>', lambda e: cancel_reset())

    # Store the confirmation handlers globally so serial listener can access them
    global reset_season_confirm_window, reset_season_confirm_handler, reset_season_cancel_handler
    reset_season_confirm_window = confirm_window
    reset_season_confirm_handler = confirm_reset
    reset_season_cancel_handler = cancel_reset

    # Start countdown
    confirm_window.after(1000, update_countdown)


def confirm_reset_flow_curve():
    """Confirm and reset learned flow curve override with OV/PS commands."""
    confirm_window = tk.Toplevel()
    confirm_window.title("Flow Curve Reset Confirmation")
    confirm_window.attributes('-fullscreen', True)
    confirm_window.configure(bg='black')

    message = tk.Label(confirm_window, text="USE FACTORY FLOW CURVE?",
                      font=("Helvetica", 46, "bold"), fg="orange", bg="black")
    message.pack(expand=True, pady=40)

    current_curve = tk.Label(
        confirm_window,
        text=f"Current Curve: {flow_curve_status_text()}",
        font=("Helvetica", 30, "bold"),
        fg="lime",
        bg="black",
    )
    current_curve.pack(pady=16)

    detail = tk.Label(
        confirm_window,
        text="This archives learned curve samples and reloads factory defaults.",
        font=("Helvetica", 24, "bold"),
        fg="white",
        bg="black",
        wraplength=1100,
        justify=tk.CENTER,
    )
    detail.pack(pady=12)

    instructions = tk.Label(confirm_window, text="Press OV to CONFIRM or PS to CANCEL",
                          font=("Helvetica", 22, "bold"), fg="cyan", bg="black")
    instructions.pack(pady=24)

    countdown_label = tk.Label(confirm_window, text="Auto-cancel in 10 seconds",
                             font=("Helvetica", 22), fg="white", bg="black")
    countdown_label.pack(pady=16)

    countdown = [10]

    def cancel_reset():
        global reset_flow_curve_confirm_window, reset_flow_curve_confirm_handler
        global reset_flow_curve_cancel_handler
        confirm_window.destroy()
        reset_flow_curve_confirm_window = None
        reset_flow_curve_confirm_handler = None
        reset_flow_curve_cancel_handler = None

    def confirm_reset():
        global reset_flow_curve_confirm_window, reset_flow_curve_confirm_handler
        global reset_flow_curve_cancel_handler
        archived = flow_curve.reset_learning(
            FLOW_CURVE_SAMPLES_PATH,
            FLOW_CURVE_PROPOSAL_PATH,
            FLOW_CURVE_OVERRIDE_PATH,
        )
        load_flow_curve_state()
        with open("/home/pi/fill_calibration.log", "a") as f:
            f.write(
                f"{time.strftime('%Y-%m-%d %H:%M:%S')} | CurveLearning: factory reset"
                f" | Archived: {', '.join(archived) if archived else 'none'}\n"
            )
        confirm_window.destroy()
        reset_flow_curve_confirm_window = None
        reset_flow_curve_confirm_handler = None
        reset_flow_curve_cancel_handler = None
        if menu_window:
            close_menu()
            show_menu()

    def update_countdown():
        countdown[0] -= 1
        if countdown[0] <= 0:
            cancel_reset()
        else:
            countdown_label.config(text=f"Auto-cancel in {countdown[0]} seconds")
            confirm_window.after(1000, update_countdown)

    confirm_window.bind('<Escape>', lambda e: cancel_reset())

    global reset_flow_curve_confirm_window, reset_flow_curve_confirm_handler
    global reset_flow_curve_cancel_handler
    reset_flow_curve_confirm_window = confirm_window
    reset_flow_curve_confirm_handler = confirm_reset
    reset_flow_curve_cancel_handler = cancel_reset

    confirm_window.after(1000, update_countdown)


def confirm_accept_flow_curve():
    """Confirm and activate a pending learned flow curve proposal."""
    proposal = flow_curve.load_curve_proposal(FLOW_CURVE_PROPOSAL_PATH)
    if not proposal:
        confirm_window = tk.Toplevel()
        confirm_window.title("No Curve Proposal")
        confirm_window.attributes('-fullscreen', True)
        confirm_window.configure(bg='black')

        message = tk.Label(confirm_window, text="NO LEARNED CURVE READY",
                          font=("Helvetica", 46, "bold"), fg="orange", bg="black")
        message.pack(expand=True, pady=50)
        detail = tk.Label(confirm_window, text="Need 3 thumbs-up confirmed Auto fills first.",
                         font=("Helvetica", 28, "bold"), fg="white", bg="black")
        detail.pack(pady=20)
        instructions = tk.Label(confirm_window, text="Press OV or PS to return",
                              font=("Helvetica", 22, "bold"), fg="cyan", bg="black")
        instructions.pack(pady=30)

        def close_notice():
            global accept_flow_curve_confirm_window, accept_flow_curve_confirm_handler
            global accept_flow_curve_cancel_handler
            confirm_window.destroy()
            accept_flow_curve_confirm_window = None
            accept_flow_curve_confirm_handler = None
            accept_flow_curve_cancel_handler = None

        global accept_flow_curve_confirm_window, accept_flow_curve_confirm_handler
        global accept_flow_curve_cancel_handler
        accept_flow_curve_confirm_window = confirm_window
        accept_flow_curve_confirm_handler = close_notice
        accept_flow_curve_cancel_handler = close_notice
        confirm_window.after(8000, close_notice)
        return

    learning = proposal.get("learning", {})
    offset = learning.get("applied_offset_gallons")
    raw_offset = learning.get("raw_offset_gallons")
    sample_count = proposal.get("sample_count", 0)

    confirm_window = tk.Toplevel()
    confirm_window.title("Accept Flow Curve Confirmation")
    confirm_window.attributes('-fullscreen', True)
    confirm_window.configure(bg='black')

    message = tk.Label(confirm_window, text="ACCEPT LEARNED FLOW CURVE?",
                      font=("Helvetica", 44, "bold"), fg="orange", bg="black")
    message.pack(expand=True, pady=36)

    detail_text = (
        f"Samples: {sample_count} Auto fills\n"
        f"Proposed offset: {offset:+.3f} gal\n"
        f"Raw offset: {raw_offset:+.3f} gal"
        if isinstance(offset, (int, float)) and isinstance(raw_offset, (int, float))
        else f"Samples: {sample_count} Auto fills"
    )
    detail = tk.Label(
        confirm_window,
        text=detail_text,
        font=("Helvetica", 28, "bold"),
        fg="lime",
        bg="black",
        justify=tk.CENTER,
    )
    detail.pack(pady=16)

    warning = tk.Label(
        confirm_window,
        text="This makes the learned curve active. Factory curve remains available.",
        font=("Helvetica", 23, "bold"),
        fg="white",
        bg="black",
        wraplength=1100,
        justify=tk.CENTER,
    )
    warning.pack(pady=12)

    instructions = tk.Label(confirm_window, text="Press OV to ACCEPT or PS to CANCEL",
                          font=("Helvetica", 22, "bold"), fg="cyan", bg="black")
    instructions.pack(pady=24)

    countdown_label = tk.Label(confirm_window, text="Auto-cancel in 10 seconds",
                             font=("Helvetica", 22), fg="white", bg="black")
    countdown_label.pack(pady=16)

    countdown = [10]

    def cancel_accept():
        global accept_flow_curve_confirm_window, accept_flow_curve_confirm_handler
        global accept_flow_curve_cancel_handler
        confirm_window.destroy()
        accept_flow_curve_confirm_window = None
        accept_flow_curve_confirm_handler = None
        accept_flow_curve_cancel_handler = None

    def confirm_accept():
        global accept_flow_curve_confirm_window, accept_flow_curve_confirm_handler
        global accept_flow_curve_cancel_handler
        accept_pending_flow_curve("TrailerScreen")
        confirm_window.destroy()
        accept_flow_curve_confirm_window = None
        accept_flow_curve_confirm_handler = None
        accept_flow_curve_cancel_handler = None
        if menu_window:
            close_menu()
            show_menu()

    def update_countdown():
        countdown[0] -= 1
        if countdown[0] <= 0:
            cancel_accept()
        else:
            countdown_label.config(text=f"Auto-cancel in {countdown[0]} seconds")
            confirm_window.after(1000, update_countdown)

    confirm_window.bind('<Escape>', lambda e: cancel_accept())

    accept_flow_curve_confirm_window = confirm_window
    accept_flow_curve_confirm_handler = confirm_accept
    accept_flow_curve_cancel_handler = cancel_accept

    confirm_window.after(1000, update_countdown)

def shutdown_system():
    """Shutdown the system"""
    import subprocess
    # Show confirmation for 2 seconds then shutdown
    confirm_window = tk.Toplevel()
    confirm_window.title("Shutdown")
    confirm_window.attributes('-fullscreen', True)
    confirm_window.configure(bg='black')

    msg = tk.Label(confirm_window, text="SHUTTING DOWN...",
                  font=("Helvetica", 48, "bold"), fg="red", bg="black")
    msg.pack(expand=True)

    def do_shutdown():
        time.sleep(2)
        # Use systemctl poweroff with password
        subprocess.run(['bash', '-c', 'echo raspi | sudo -S systemctl poweroff'])

    shutdown_thread = threading.Thread(target=do_shutdown, daemon=True)
    shutdown_thread.start()

def reboot_system():
    """Reboot the system"""
    import subprocess
    # Show confirmation for 2 seconds then reboot
    confirm_window = tk.Toplevel()
    confirm_window.title("Reboot")
    confirm_window.attributes('-fullscreen', True)
    confirm_window.configure(bg='black')

    msg = tk.Label(confirm_window, text="REBOOTING...",
                  font=("Helvetica", 48, "bold"), fg="orange", bg="black")
    msg.pack(expand=True)

    def do_reboot():
        time.sleep(2)
        # Use systemctl reboot with password
        subprocess.run(['bash', '-c', 'echo raspi | sudo -S systemctl reboot'])

    reboot_thread = threading.Thread(target=do_reboot, daemon=True)
    reboot_thread.start()

def style_menu_item(index, selected):
    """Apply selected or unselected styling to a single menu item."""
    if index < 0 or index >= len(menu_buttons) or index >= len(menu_arrows):
        return

    btn = menu_buttons[index]
    arrow = menu_arrows[index]

    # High-contrast unselected palette for sunlight readability.
    colors = [
        ("#d7efff", "#111111"),  # View Logs
        ("#eadcff", "#111111"),  # Fill History
        ("#d8ffff", "#111111"),  # Tank Calibration
        ("#ffe3bf", "#111111"),  # Full Test
        ("#dff7d9", "#111111"),  # Reset Season
        ("#e7ffd8", "#111111"),  # Self Test
        ("#fff0b8", "#111111"),  # Capture Bug
        ("#eadcff", "#111111"),  # System Update
        ("#ffd6d6", "#111111"),  # Shutdown
        ("#ffe3bf", "#111111"),  # Reboot
        ("#ffc9c9", "#111111"),  # Exit to Desktop
        ("#e3e3e3", "#111111"),  # Exit Menu
    ]

    if selected:
        # SELECTED - Use a dark tile with bright text so it stands off the lighter menu items.
        btn.config(bg="#111111", fg="#ffff00",
                  activebackground="#111111", activeforeground="#ffff00",
                  font=("Helvetica", 28, "bold"),
                  relief=tk.RAISED, borderwidth=6,
                  highlightbackground="#ffffff", highlightthickness=4, highlightcolor="#ffffff",
                  width=16, height=2,
                  wraplength=360, justify=tk.CENTER)
        arrow.config(text=">>> SELECTED >>>", fg="#00ffff",
                    font=("Helvetica", 20, "bold"))
    else:
        bg, fg = colors[index % len(colors)]
        btn.config(bg=bg, fg=fg,
                  activebackground=bg, activeforeground=fg,
                  font=("Helvetica", 28, "bold"),
                  relief=tk.FLAT, borderwidth=2,
                  highlightthickness=0,
                  highlightbackground=bg,
                  highlightcolor=bg,
                  width=16, height=2,
                  wraplength=360, justify=tk.CENTER)
        arrow.config(text="", fg="black")

def update_menu_highlight(full_refresh=False):
    """Update visual highlighting of selected menu item."""
    global menu_buttons, menu_arrows, menu_selected_index, menu_position_label, menu_displayed_index

    if not menu_buttons or not menu_arrows:
        return

    # Update position indicator with current selection
    if menu_position_label:
        menu_position_label.config(
            text=f"Option {menu_selected_index + 1} of {len(MENU_ITEMS)}: {MENU_ITEMS[menu_selected_index]}"
        )

    if full_refresh or menu_displayed_index is None:
        for i in range(len(menu_buttons)):
            style_menu_item(i, i == menu_selected_index)
    else:
        if menu_displayed_index != menu_selected_index:
            style_menu_item(menu_displayed_index, False)
        style_menu_item(menu_selected_index, True)

    menu_displayed_index = menu_selected_index

def _apply_menu_highlight_update():
    """Run one coalesced menu highlight redraw on the Tk thread."""
    global menu_highlight_refresh_pending
    menu_highlight_refresh_pending = False
    update_menu_highlight()

def schedule_menu_highlight_update():
    """Schedule at most one menu redraw while serial knob pulses are arriving."""
    global menu_highlight_refresh_pending
    if menu_highlight_refresh_pending:
        return
    menu_highlight_refresh_pending = True
    root.after(0, _apply_menu_highlight_update)


def arm_menu_ov_guard():
    """Ignore repeated OV messages from the same press after a screen change."""
    global menu_ov_guard_until
    menu_ov_guard_until = time.time() + MENU_OV_GUARD_SECONDS


def should_ignore_menu_ov_bounce(line, source):
    global menu_ov_guard_until
    if line != "OV":
        menu_ov_guard_until = 0.0
        return False
    if time.time() >= menu_ov_guard_until:
        return False
    msg = f"{source}: Ignored OV bounce after menu select"
    print(msg)
    log_serial_debug(msg)
    append_debug_log(debug_log, f"{time.strftime('%Y-%m-%d %H:%M:%S')} - {msg}\n")
    return True


def menu_navigate_up():
    """Move selection up in menu"""
    global menu_selected_index
    old_index = menu_selected_index
    menu_selected_index = (menu_selected_index - 1) % len(MENU_ITEMS)
    append_debug_log('/home/pi/menu_debug.log', f"{time.strftime('%Y-%m-%d %H:%M:%S')} - menu_navigate_up: {old_index} -> {menu_selected_index}\n")
    schedule_menu_highlight_update()

def menu_navigate_down():
    """Move selection down in menu"""
    global menu_selected_index
    old_index = menu_selected_index
    menu_selected_index = (menu_selected_index + 1) % len(MENU_ITEMS)
    append_debug_log('/home/pi/menu_debug.log', f"{time.strftime('%Y-%m-%d %H:%M:%S')} - menu_navigate_down: {old_index} -> {menu_selected_index}\n")
    schedule_menu_highlight_update()

def menu_select():
    """Activate the currently selected menu item"""
    global menu_selected_index

    # Debug logging to track selection
    append_debug_log('/home/pi/menu_debug.log', f"{time.strftime('%Y-%m-%d %H:%M:%S')} - menu_select() called with index={menu_selected_index}, item={MENU_ITEMS[menu_selected_index] if menu_selected_index < len(MENU_ITEMS) else 'UNKNOWN'}\n")

    if menu_selected_index == 0:
        show_log_viewer()
    elif menu_selected_index == 1:
        show_fill_history()
    elif menu_selected_index == 2:
        show_tank_calibration()
    elif menu_selected_index == 3:
        run_full_test()
    elif menu_selected_index == 4:
        confirm_reset_season()
    elif menu_selected_index == 5:
        confirm_accept_flow_curve()
    elif menu_selected_index == 6:
        confirm_reset_flow_curve()
    elif menu_selected_index == 7:
        run_self_test()
    elif menu_selected_index == 8:
        run_bug_capture()
    elif menu_selected_index == 9:
        run_system_update()
    elif menu_selected_index == 10:
        shutdown_system()
    elif menu_selected_index == 11:
        reboot_system()
    elif menu_selected_index == 12:
        exit_to_desktop()
    elif menu_selected_index == 13:
        close_menu()

def exit_to_desktop():
    """Exit the dashboard and return to desktop with confirmation"""
    # Create confirmation window
    confirm_window = tk.Toplevel()
    confirm_window.title("Exit Confirmation")
    confirm_window.attributes('-fullscreen', True)
    confirm_window.configure(bg='black')
    
    # Confirmation message
    message = tk.Label(confirm_window, text="EXIT TO DESKTOP?", 
                       font=("Helvetica", 48, "bold"), fg="red", bg="black")
    message.pack(expand=True, pady=50)
    
    # Instructions
    instructions = tk.Label(confirm_window, text="Press OV to CONFIRM or PS to CANCEL",
                           font=("Helvetica", 22, "bold"), fg="cyan", bg="black")
    instructions.pack(pady=30)
    
    # Countdown timer
    countdown_label = tk.Label(confirm_window, text="Auto-cancel in 10 seconds",
                              font=("Helvetica", 22), fg="white", bg="black")
    countdown_label.pack(pady=20)
    
    countdown = [10]  # Use list to allow modification in nested function
    
    def cancel_exit():
        global exit_confirm_window, exit_confirm_handler, exit_cancel_handler
        confirm_window.destroy()
        exit_confirm_window = None
        exit_confirm_handler = None
        exit_cancel_handler = None
        # Menu window is already open behind this dialog - just let it reappear
    
    def confirm_exit():
        print("Exiting dashboard to desktop...")
        confirm_window.destroy()
        root.destroy()
        sys.exit(0)
    
    def update_countdown():
        countdown[0] -= 1
        if countdown[0] <= 0:
            cancel_exit()
        else:
            countdown_label.config(text=f"Auto-cancel in {countdown[0]} seconds")
            confirm_window.after(1000, update_countdown)
    
    # Bind keyboard shortcuts for confirmation
    confirm_window.bind('<Escape>', lambda e: cancel_exit())
    
    # Store the confirmation handlers globally so serial listener can access them
    global exit_confirm_window, exit_confirm_handler, exit_cancel_handler
    exit_confirm_window = confirm_window
    exit_confirm_handler = confirm_exit
    exit_cancel_handler = cancel_exit
    
    # Start countdown
    confirm_window.after(1000, update_countdown)

def close_menu():
    """Close the menu and return to main dashboard"""
    global menu_mode, menu_window, menu_buttons, menu_arrows, menu_selected_index, menu_displayed_index
    global menu_highlight_refresh_pending
    global exit_confirm_window, exit_confirm_handler, exit_cancel_handler
    global reset_flow_curve_confirm_window, reset_flow_curve_confirm_handler
    global reset_flow_curve_cancel_handler
    global accept_flow_curve_confirm_window, accept_flow_curve_confirm_handler
    global accept_flow_curve_cancel_handler
    menu_mode = False
    menu_selected_index = 0
    menu_displayed_index = None
    menu_highlight_refresh_pending = False
    menu_buttons = []
    menu_arrows = []
    # Reset exit confirmation globals
    exit_confirm_window = None
    exit_confirm_handler = None
    exit_cancel_handler = None
    reset_flow_curve_confirm_window = None
    reset_flow_curve_confirm_handler = None
    reset_flow_curve_cancel_handler = None
    accept_flow_curve_confirm_window = None
    accept_flow_curve_confirm_handler = None
    accept_flow_curve_cancel_handler = None
    if menu_window:
        menu_window.destroy()
        menu_window = None

def dismiss_reminders():
    """Dismiss the daily reminders screen"""
    global reminders_mode, reminders_window, last_reminder_date
    reminders_mode = False
    # Mark today as shown
    last_reminder_date = time.strftime('%Y-%m-%d')
    if reminders_window:
        reminders_window.destroy()
        reminders_window = None
    print(f"Daily reminders dismissed for {last_reminder_date}")

def show_daily_reminders():
    """Display daily reminders checklist with time-based greeting"""
    global reminders_mode, reminders_window

    reminders_mode = True
    reminders_window = tk.Toplevel(root)
    reminders_window.title("Daily Reminders")
    reminders_window.attributes('-fullscreen', True)
    reminders_window.configure(bg='black')

    # Determine greeting based on time of day
    current_hour = time.localtime().tm_hour
    if 6 <= current_hour < 12:
        greeting = "☀️ GOOD MORNING! ☀️"
        color = "yellow"
    elif 12 <= current_hour < 18:
        greeting = "🌤️ GOOD AFTERNOON! 🌤️"
        color = "orange"
    else:
        greeting = "🌙 GOOD EVENING! 🌙"
        color = "lightblue"

    # Title with time-based greeting
    title = tk.Label(reminders_window, text=greeting, font=("Helvetica", 48, "bold"),
                     fg=color, bg="black")
    title.pack(pady=30)

    # Subtitle
    subtitle = tk.Label(reminders_window, text="Before you start:", font=("Helvetica", 32),
                       fg="cyan", bg="black")
    subtitle.pack(pady=10)

    # Reminders frame
    reminders_frame = tk.Frame(reminders_window, bg='black')
    reminders_frame.pack(expand=True, pady=20)

    # Reminder items
    reminders = [
        "✓ Check for water in fuel",
        "✓ Connect app",
        "✓ Stay safe!"
    ]

    for reminder in reminders:
        label = tk.Label(reminders_frame, text=reminder, font=("Helvetica", 36, "bold"),
                        fg="white", bg="black", anchor="w")
        label.pack(pady=15, padx=50)

    # Instructions at bottom
    instructions = tk.Label(reminders_window, text="Press THUMBS UP button or send OV to continue",
                           font=("Helvetica", 28, "bold"), fg="green", bg="black")
    instructions.pack(side=tk.BOTTOM, pady=30)

def show_menu():
    """Display the main menu"""
    global menu_mode, menu_window, menu_buttons, menu_arrows, menu_selected_index, menu_position_label
    global menu_displayed_index, menu_highlight_refresh_pending

    load_flow_curve_state()

    # Defensive cleanup in case any stale submenu window/mode was left behind.
    global log_viewer_mode, log_viewer_window, log_viewer_text
    global fill_history_mode, fill_history_window, fill_history_text
    global calibration_mode, calibration_window, calibration_state
    global reset_flow_curve_confirm_window, reset_flow_curve_confirm_handler
    global reset_flow_curve_cancel_handler
    global accept_flow_curve_confirm_window, accept_flow_curve_confirm_handler
    global accept_flow_curve_cancel_handler

    log_viewer_mode = False
    if log_viewer_window:
        try:
            log_viewer_window.destroy()
        except Exception:
            pass
        log_viewer_window = None
        log_viewer_text = None

    fill_history_mode = False
    if fill_history_window:
        try:
            fill_history_window.destroy()
        except Exception:
            pass
        fill_history_window = None
        fill_history_text = None

    calibration_mode = False
    if calibration_window:
        try:
            calibration_window.destroy()
        except Exception:
            pass
        calibration_window = None
        calibration_state = None

    if reset_flow_curve_confirm_window:
        try:
            reset_flow_curve_confirm_window.destroy()
        except Exception:
            pass
        reset_flow_curve_confirm_window = None
        reset_flow_curve_confirm_handler = None
        reset_flow_curve_cancel_handler = None

    if accept_flow_curve_confirm_window:
        try:
            accept_flow_curve_confirm_window.destroy()
        except Exception:
            pass
        accept_flow_curve_confirm_window = None
        accept_flow_curve_confirm_handler = None
        accept_flow_curve_cancel_handler = None

    if menu_window:
        try:
            menu_window.destroy()
        except Exception:
            pass
        menu_window = None

    menu_mode = True
    menu_selected_index = 0  # Start at first item
    menu_displayed_index = None
    menu_highlight_refresh_pending = False
    menu_buttons = []
    menu_arrows = []

    menu_window = tk.Toplevel(root)
    menu_window.title("System Menu")
    menu_window.attributes('-fullscreen', True)
    menu_window.configure(bg='#0a0a0a')

    # ═══════════════════════════════════════════════════════════════════
    # TOP INFO BAR - Professional header with system info
    # ═══════════════════════════════════════════════════════════════════
    header_frame = tk.Frame(menu_window, bg='#1a1a1a', highlightbackground='#333333',
                           highlightthickness=2)
    header_frame.pack(fill=tk.X, padx=10, pady=5)

    # Left side - System Information Panel
    left_info_frame = tk.Frame(header_frame, bg='#1a1a1a')
    left_info_frame.pack(side=tk.LEFT, padx=15, pady=8)

    # WiFi status indicator
    wifi_status = check_wifi_status()
    if "CONNECTED" in wifi_status:
        wifi_color = "#00ff00"  # Bright green
        wifi_symbol = "●"
    else:
        wifi_color = "#ff0000"  # Bright red
        wifi_symbol = "●"

    wifi_label = tk.Label(left_info_frame, text=f"{wifi_symbol} {wifi_status}",
                         font=("Helvetica", 24, "bold"), fg=wifi_color, bg="#1a1a1a")
    wifi_label.pack(anchor='w')

    # IP address
    ip_address = get_ip_address()
    ip_label = tk.Label(left_info_frame, text=f"IP: {ip_address}",
                       font=("Helvetica", 22), fg="#00d4ff", bg="#1a1a1a")
    ip_label.pack(anchor='w', pady=2)

    # Assigned trailer
    assigned_trailer = get_assigned_trailer_label()
    trailer_label = tk.Label(left_info_frame, text=f"Trailer: {assigned_trailer}",
                             font=("Helvetica", 22), fg="#00d4ff", bg="#1a1a1a")
    trailer_label.pack(anchor='w', pady=2)

    # Username
    username = get_username()
    user_label = tk.Label(left_info_frame, text=f"User: {username}",
                         font=("Helvetica", 22), fg="#00d4ff", bg="#1a1a1a")
    user_label.pack(anchor='w', pady=2)

    # Center - Title and Version
    center_frame = tk.Frame(header_frame, bg='#1a1a1a')
    center_frame.pack(side=tk.LEFT, expand=True, padx=15, pady=8)

    title = tk.Label(center_frame, text="SYSTEM MENU", font=("Helvetica", 32, "bold"),
                     fg="#00d4ff", bg="#1a1a1a")
    title.pack()

    version_label = tk.Label(center_frame, text=f"Version {VERSION}", font=("Helvetica", 20),
                            fg="#888888", bg="#1a1a1a")
    version_label.pack()

    curve_label = tk.Label(center_frame, text=f"Curve: {flow_curve_status_text()}",
                           font=("Helvetica", 18), fg="#ffaa00", bg="#1a1a1a")
    curve_label.pack()

    proposal_label = tk.Label(center_frame, text=flow_curve_proposal_status_text(),
                              font=("Helvetica", 16), fg="#ffd080", bg="#1a1a1a")
    proposal_label.pack()

    # Right side - Totals Panel
    global menu_daily_label, menu_season_label
    right_info_frame = tk.Frame(header_frame, bg='#1a1a1a')
    right_info_frame.pack(side=tk.RIGHT, padx=15, pady=8)

    totals_title = tk.Label(right_info_frame, text="TOTALS",
                           font=("Helvetica", 22, "bold"), fg="#888888", bg="#1a1a1a")
    totals_title.pack()

    menu_daily_label = tk.Label(right_info_frame, text=f"Daily: {daily_total:.1f} gal",
                          font=("Helvetica", 24, "bold"), fg="#00ffff", bg="#1a1a1a")
    menu_daily_label.pack(anchor='e', pady=2)

    menu_season_label = tk.Label(right_info_frame, text=f"Season: {season_total:.1f} gal",
                           font=("Helvetica", 24, "bold"), fg="#00ff00", bg="#1a1a1a")
    menu_season_label.pack(anchor='e', pady=2)

    # ═══════════════════════════════════════════════════════════════════
    # MENU OPTIONS SECTION
    # ═══════════════════════════════════════════════════════════════════

    # Position indicator - make it global so we can update it
    global menu_position_label
    menu_position_label = tk.Label(menu_window,
                                    text=f"Option 1 of {len(MENU_ITEMS)}: {MENU_ITEMS[0]}",
                                    font=("Helvetica", 18, "bold"),
                                    fg="#ffffff", bg='#0a0a0a')
    menu_position_label.pack(pady=2)

    # Menu buttons frame with two-column layout for larger text.
    button_frame = tk.Frame(menu_window, bg='#0a0a0a')
    button_frame.pack(expand=True, fill=tk.BOTH, padx=24, pady=6)
    button_frame.grid_columnconfigure(0, weight=1, uniform="menu")
    button_frame.grid_columnconfigure(1, weight=1, uniform="menu")
    for row in range((len(MENU_ITEMS) + 1) // 2):
        button_frame.grid_rowconfigure(row, weight=1, uniform="menu_row")

    menu_actions = [
        ("VIEW LOGS", show_log_viewer),
        ("FILL HISTORY", show_fill_history),
        ("TANK CALIBRATION", show_tank_calibration),
        ("FULL TEST", run_full_test),
        ("RESET SEASON", lambda: confirm_reset_season()),
        ("ACCEPT CURVE", lambda: confirm_accept_flow_curve()),
        ("FACTORY CURVE", lambda: confirm_reset_flow_curve()),
        ("SELF TEST", run_self_test),
        ("CAPTURE BUG", run_bug_capture),
        ("SYSTEM UPDATE", run_system_update),
        ("SHUTDOWN", shutdown_system),
        ("REBOOT", reboot_system),
        ("EXIT TO DESKTOP", exit_to_desktop),
        ("EXIT MENU", close_menu),
    ]

    for index, (label, action) in enumerate(menu_actions):
        row = index // 2
        column = index % 2

        cell = tk.Frame(button_frame, bg='#0a0a0a')
        cell.grid(row=row, column=column, sticky="nsew", padx=12, pady=8)
        cell.grid_columnconfigure(0, weight=1)
        cell.grid_rowconfigure(1, weight=1)

        arrow = tk.Label(
            cell,
            text="",
            font=("Helvetica", 20, "bold"),
            fg="#00ffff",
            bg="#0a0a0a",
        )
        arrow.grid(row=0, column=0, pady=(0, 4))

        btn = tk.Button(
            cell,
            text=label,
            font=("Helvetica", 28, "bold"),
            bg="white",
            fg="black",
            command=action,
            width=16,
            height=2,
            borderwidth=3,
            wraplength=360,
            justify=tk.CENTER,
        )
        btn.grid(row=1, column=0, sticky="nsew")

        menu_buttons.append(btn)
        menu_arrows.append(arrow)

    # Instructions - Professional footer
    footer_frame = tk.Frame(menu_window, bg='#1a1a1a', highlightbackground='#333333',
                           highlightthickness=2)
    footer_frame.pack(side=tk.BOTTOM, fill=tk.X, padx=10, pady=5)

    instructions = tk.Label(footer_frame, text="+1 = NEXT  │  -1 = PREVIOUS  │  OV = SELECT",
                           font=("Helvetica", 20, "bold"), fg="#00d4ff", bg="#1a1a1a")
    instructions.pack(pady=8)

    # Apply initial highlight
    update_menu_highlight(full_refresh=True)

def iol_power_cycle():
    """Power-cycle the IOL port in a background thread to trigger re-negotiation.

    Called when the flow meter is detected as disconnected (all-zero or stale data).
    The IOL master firmware does not auto-negotiate when a device is reconnected,
    so the port must be powered off and back on to restart the IO-Link handshake.
    Uses readStatus2() to check hardware error state before and after the cycle.
    """
    global iol_power_cycle_in_progress, last_power_cycle_time

    try:
        with iol_io_lock:
            # Check port status before power cycle
            try:
                pre_status = iolhat.readStatus2(config.IOL_PORT)
                print(f"IOL power-cycle: Pre-cycle status - pdInValid={pre_status.pd_in_valid}, "
                      f"txRate=0x{pre_status.transmission_rate:02X}, "
                      f"error=0x{pre_status.error:02X}, power={pre_status.power}", flush=True)
            except Exception as e:
                print(f"IOL power-cycle: Pre-cycle status check failed: {e}", flush=True)

            print(f"IOL power-cycle: Starting port {config.IOL_PORT} power-cycle", flush=True)

            # Step 1: Power off
            try:
                iolhat.power(config.IOL_PORT, 0)
                print(f"IOL power-cycle: Port {config.IOL_PORT} powered OFF", flush=True)
            except Exception as e:
                print(f"IOL power-cycle: Failed to power off: {e}", flush=True)
                return

            time.sleep(1.0)

            # Step 2: Power on
            try:
                iolhat.power(config.IOL_PORT, 1)
                print(f"IOL power-cycle: Port {config.IOL_PORT} powered ON", flush=True)
            except Exception as e:
                print(f"IOL power-cycle: Failed to power on: {e}", flush=True)
                return

            # Step 3: Wait for IO-Link handshake (up to 5 seconds, polling status)
            for attempt in range(5):
                time.sleep(1.0)
                try:
                    post_status = iolhat.readStatus2(config.IOL_PORT)
                    print(f"IOL power-cycle: Post-cycle check {attempt+1}/5 - "
                          f"pdInValid={post_status.pd_in_valid}, "
                          f"txRate=0x{post_status.transmission_rate:02X}, "
                          f"error=0x{post_status.error:02X}", flush=True)
                    if post_status.pd_in_valid == 1 and post_status.transmission_rate != 0:
                        print(f"IOL power-cycle: Device reconnected successfully", flush=True)
                        try:
                            iolhat.led(config.IOL_PORT, iolhat.LED_GREEN)
                        except:
                            pass
                        return
                except Exception as e:
                    print(f"IOL power-cycle: Post-cycle status check failed: {e}", flush=True)

        print(f"IOL power-cycle: Device did not reconnect after power cycle", flush=True)

    except Exception as e:
        print(f"IOL power-cycle: Unexpected error: {e}", flush=True)
    finally:
        last_power_cycle_time = time.time()
        iol_power_cycle_in_progress = False

def _try_iol_power_cycle():
    """Check rate-limiting and spawn power-cycle thread if appropriate."""
    global iol_power_cycle_in_progress

    if iol_power_cycle_in_progress:
        return

    if (time.time() - last_power_cycle_time) < config.IOL_RECONNECT_INTERVAL:
        return

    iol_power_cycle_in_progress = True
    cycle_thread = threading.Thread(target=iol_power_cycle, daemon=True)
    cycle_thread.start()


def _log_iol_disconnect_status(reason):
    """Read STATUS2 and log the hardware error byte when a disconnect is first detected."""
    try:
        _, st = _read_iol_status_ok()
        print(f"IOL DISCONNECT [{reason}]: pdInValid={st.pd_in_valid}, "
              f"txRate=0x{st.transmission_rate:02X}, "
              f"cycleTime=0x{st.master_cycle_time:02X}, "
              f"error=0x{st.error:02X}, power={st.power}", flush=True)
    except Exception as e:
        print(f"IOL DISCONNECT [{reason}]: STATUS2 read failed: {e}", flush=True)

_iol_port_power_restored_at = 0.0


def _read_iol_status_ok():
    """Return whether IO-Link status indicates valid process data."""
    global _iol_port_power_restored_at
    with iol_io_lock:
        st = iolhat.readStatus2(config.IOL_PORT)
        # The IOL master leaves the port's L+ switch wherever it was; after the
        # master is relaunched by its supervisor it comes up OFF, the MAX14819
        # then reports UVL+ (0 V on the open pin) and this check would report
        # "LOW VOLTAGE" forever while data is flowing. Turn L+ back on and
        # re-read (rate-limited so a genuine 24 V fault is not hammered).
        if getattr(st, "power", 1) != 1 and time.time() - _iol_port_power_restored_at > 5.0:
            _iol_port_power_restored_at = time.time()
            try:
                iolhat.power(config.IOL_PORT, 1)
                print("IOL port power was OFF (master restarted?) - switched L+ back on", flush=True)
                st = iolhat.readStatus2(config.IOL_PORT)
            except Exception as e:
                print(f"IOL port power restore failed: {e}", flush=True)
    ok = st.pd_in_valid == 1 and st.transmission_rate != 0 and st.error == 0
    return ok, st


def _describe_iol_status_fault(st):
    """Return a field-friendly reason for an unhealthy IO-Link status."""
    status_error = getattr(st, "error", 0)
    if status_error & 0x02:
        return "LOW VOLTAGE: check 24V"
    if status_error & 0x04:
        return "IO-Link current limit"
    if status_error & 0x01:
        return "IO-Link CQ fault"
    if getattr(st, "power", 0) != 1:
        return "IO-Link port power off"
    if getattr(st, "pd_in_valid", 0) != 1:
        return "flow meter data invalid"
    if getattr(st, "transmission_rate", 0) == 0:
        return "flow meter no COM"
    return "waiting for healthy flow meter status"


def read_flow_meter():
    """Read data from the Picomag flow meter via IO-Link"""
    global last_totalizer_liters, last_signed_totalizer_liters, last_flow_rate
    global connection_error, error_message, last_successful_read_time
    global last_flow_meter_temp_f
    global last_flow_read_was_fresh, last_fresh_flow_read_time
    global consecutive_identical_raw, last_raw_data

    try:
        last_flow_read_was_fresh = False
        # Read process data from IO-Link device. The flow-control thread is the
        # normal owner; this lock protects occasional diagnostics from racing it.
        with iol_io_lock:
            raw_data = iolhat.pd(config.IOL_PORT, 0, config.DATA_LENGTH, None)

        if len(raw_data) >= 15:
            if raw_data == b'\x00' * len(raw_data):
                if not connection_error:
                    _log_iol_disconnect_status("all-zero data")
                connection_error = True
                error_message = "Device not responding (all-zero data)"
                try:
                    iolhat.led(config.IOL_PORT, iolhat.LED_RED)
                except:
                    pass
                last_raw_data = None
                consecutive_identical_raw = 0
                _try_iol_power_cycle()
                return last_totalizer_liters * config.LITERS_TO_GALLONS

            # If we were in error state and now getting valid non-stale data, log recovery
            if connection_error and raw_data != last_raw_data:
                print(f"Flow meter reconnected - valid data received", flush=True)

            raw_data_changed = raw_data != last_raw_data
            raw_flow_rate_l_per_s = struct.unpack('>f', raw_data[8:12])[0]

            # Identical idle process data can be valid at zero flow. Only treat
            # repeated identical bytes as stale if IO-Link status is unhealthy
            # or the frozen data claims flow is still active.
            if not raw_data_changed:
                consecutive_identical_raw += 1
                stale_threshold = stale_raw_threshold_reads()
                if consecutive_identical_raw >= stale_threshold:
                    try:
                        status_ok, st = _read_iol_status_ok()
                    except Exception:
                        status_ok = False
                        st = None

                    if status_ok and abs(raw_flow_rate_l_per_s) < config.FLOW_METER_ZERO_THRESHOLD:
                        if consecutive_identical_raw == stale_threshold:
                            print("Flow meter identical idle data accepted with healthy IO-Link status", flush=True)
                            log_flow_control(
                                "flow_meter_identical_data_ok"
                                f" | count={consecutive_identical_raw}"
                                f" | flow_lps={raw_flow_rate_l_per_s:.6f}"
                                f" | pdInValid={st.pd_in_valid}"
                                f" | txRate=0x{st.transmission_rate:02X}"
                                f" | error=0x{st.error:02X}"
                            )
                        consecutive_identical_raw = 0
                    else:
                        if st is not None and consecutive_identical_raw == stale_threshold:
                            log_flow_control(
                                "flow_meter_identical_data_bad_status"
                                f" | count={consecutive_identical_raw}"
                                f" | flow_lps={raw_flow_rate_l_per_s:.6f}"
                                f" | pdInValid={st.pd_in_valid}"
                                f" | txRate=0x{st.transmission_rate:02X}"
                                f" | error=0x{st.error:02X}"
                            )
                        connection_error = True
                        stale_secs = consecutive_identical_raw * flow_read_interval_seconds()
                        error_message = f"Stale data - meter may be disconnected ({stale_secs:.0f}s)"
                        if consecutive_identical_raw == stale_threshold:
                            print(f"Flow meter stale data detected after {stale_secs:.0f}s", flush=True)
                            _log_iol_disconnect_status("stale data")
                            try:
                                iolhat.led(config.IOL_PORT, iolhat.LED_RED)
                            except:
                                pass
                        _try_iol_power_cycle()
                        return last_totalizer_liters * config.LITERS_TO_GALLONS
            else:
                if consecutive_identical_raw >= stale_raw_threshold_reads():
                    print(f"Flow meter data flowing again", flush=True)
                consecutive_identical_raw = 0
            last_raw_data = raw_data

            # Decode the data according to Picomag format. Keep the signed
            # totalizer so negative idle drift becomes a fault instead of a
            # plausible positive fill volume.
            signed_totalizer_liters = struct.unpack('>f', raw_data[4:8])[0]
            update_negative_totalizer_fault(signed_totalizer_liters)
            update_negative_flow_fault(signed_totalizer_liters, raw_flow_rate_l_per_s)
            update_positive_drift_fault(signed_totalizer_liters, raw_flow_rate_l_per_s)
            totalizer_liters = signed_totalizer_liters
            flow_rate_l_per_s = raw_flow_rate_l_per_s
            flow_meter_temp_f = _decode_flow_meter_temp_f(raw_data)
            detect_totalizer_reset(totalizer_liters, flow_rate_l_per_s)

            last_totalizer_liters = totalizer_liters
            last_signed_totalizer_liters = signed_totalizer_liters
            last_flow_rate = flow_rate_l_per_s
            last_flow_meter_temp_f = flow_meter_temp_f
            # Clear error state - set LED green on reconnect
            if connection_error:
                try:
                    iolhat.led(config.IOL_PORT, iolhat.LED_GREEN)
                except:
                    pass
            connection_error = False
            error_message = ""
            last_successful_read_time = time.time()
            last_flow_read_was_fresh = raw_data_changed
            if raw_data_changed:
                last_fresh_flow_read_time = last_successful_read_time

            return totalizer_liters * config.LITERS_TO_GALLONS
        else:
            connection_error = True
            error_message = "Invalid data length"
            return last_totalizer_liters * config.LITERS_TO_GALLONS

    except Exception as e:
        connection_error = True
        error_message = str(e)
        _try_iol_power_cycle()
        return last_totalizer_liters * config.LITERS_TO_GALLONS


def _flow_control_process_sample(actual, now, loop_dt_ms):
    """Run safety-critical auto-shutoff decisions from the control loop."""
    global flow_control_was_flowing, flow_cycle_counter, last_alert_triggered
    global auto_shutoff_latched, last_flowing_rate_l_per_s
    global last_trigger_flow_gpm, last_trigger_threshold, last_trigger_actual
    global last_trigger_predicted_actual, last_trigger_loop_dt_ms
    global last_pump_stop_relay_activated_at

    is_flowing = last_flow_rate >= config.FLOW_STOPPED_THRESHOLD
    flow_meter_disconnected = (time.time() - last_successful_read_time) > config.FLOW_METER_TIMEOUT
    current_flow_gpm = max(0.0, last_flow_rate * config.LITERS_PER_SEC_TO_GPM)

    if is_flowing and not flow_control_was_flowing:
        flow_cycle_counter += 1
        last_alert_triggered = False
        auto_shutoff_latched = False
        last_trigger_flow_gpm = 0.0
        last_trigger_threshold = 0.0
        last_trigger_actual = 0.0
        last_trigger_predicted_actual = 0.0
        last_trigger_loop_dt_ms = 0.0
        last_pump_stop_relay_activated_at = 0.0
        recent_flow_rates_l_per_s.clear()
        log_flow_control(
            f"cycle_start | actual={actual:.3f} | flow_lps={last_flow_rate:.4f}"
        )

    if is_flowing:
        if last_flow_read_was_fresh:
            recent_flow_rates_l_per_s.append(last_flow_rate)
            last_flowing_rate_l_per_s = last_flow_rate
    elif flow_control_was_flowing:
        log_flow_control(
            f"cycle_stop | actual={actual:.3f} | auto={auto_shutoff_latched}"
        )

    flow_control_was_flowing = is_flowing
    _update_relay_slowdown_watch(is_flowing, current_flow_gpm, now)

    if negative_totalizer_fault_active or negative_flow_fault_active:
        return
    if positive_drift_fault_active and not override_mode:
        return
    if not is_flowing or override_mode or flow_meter_disconnected or last_alert_triggered:
        return

    smoothed_flow_rate_l_per_s = get_smoothed_flow_rate()
    flow_rate_gpm = smoothed_flow_rate_l_per_s * config.LITERS_PER_SEC_TO_GPM
    trigger_threshold = calculate_trigger_threshold(smoothed_flow_rate_l_per_s)
    prediction_seconds = max(0.0, float(getattr(config, "FLOW_CONTROL_PREDICTION_SECONDS", 0.0)))
    predicted_actual = actual + (smoothed_flow_rate_l_per_s * prediction_seconds * config.LITERS_TO_GALLONS)

    if predicted_actual >= requested_gallons - trigger_threshold:
        last_alert_triggered = True
        auto_shutoff_latched = True
        last_trigger_flow_gpm = flow_rate_gpm
        last_trigger_threshold = trigger_threshold
        last_trigger_actual = actual
        last_trigger_predicted_actual = predicted_actual
        last_trigger_loop_dt_ms = loop_dt_ms
        log_flow_control(
            "auto_stop"
            f" | requested={requested_gallons:.3f}"
            f" | actual={actual:.3f}"
            f" | predicted={predicted_actual:.3f}"
            f" | threshold={trigger_threshold:.3f}"
            f" | flow_gpm={flow_rate_gpm:.1f}"
            f" | loop_dt_ms={loop_dt_ms:.1f}"
        )
        print(
            f"Auto-alert(control): Flow={flow_rate_gpm:.1f} GPM, "
            f"threshold={trigger_threshold:.2f}gal, triggering relay for "
            f"{config.AUTO_ALERT_DURATION}s"
        )
        _arm_relay_slowdown_watch(flow_rate_gpm, now)
        start_pump_stop_thread(config.AUTO_ALERT_DURATION)


def _arm_relay_slowdown_watch(flow_gpm, now):
    """Watch for measurable flow slowdown after an auto-stop relay trigger."""
    global relay_slowdown_watch_active, relay_slowdown_alarm_active
    global relay_slowdown_trigger_time, relay_slowdown_trigger_flow_gpm

    relay_slowdown_watch_active = True
    relay_slowdown_alarm_active = False
    relay_slowdown_trigger_time = now
    relay_slowdown_trigger_flow_gpm = max(0.0, flow_gpm)
    log_flow_control(
        "slowdown_watch_start"
        f" | flow_gpm={relay_slowdown_trigger_flow_gpm:.1f}"
        f" | check_after={config.RELAY_SLOWDOWN_CHECK_SECONDS:.1f}s"
    )


def _clear_relay_slowdown_watch(reason):
    """Clear post-relay slowdown watch/alarm state."""
    global relay_slowdown_watch_active, relay_slowdown_alarm_active
    global relay_slowdown_trigger_time, relay_slowdown_trigger_flow_gpm

    if relay_slowdown_watch_active or relay_slowdown_alarm_active:
        log_flow_control(f"slowdown_watch_clear | reason={reason}")
    relay_slowdown_watch_active = False
    relay_slowdown_alarm_active = False
    relay_slowdown_trigger_time = 0.0
    relay_slowdown_trigger_flow_gpm = 0.0


def _update_relay_slowdown_watch(is_flowing, flow_gpm, now):
    """Latch an alarm if flow does not drop measurably after auto-stop."""
    global relay_slowdown_alarm_active

    if not relay_slowdown_watch_active:
        return

    if not is_flowing:
        _clear_relay_slowdown_watch("flow_stopped")
        return

    elapsed = now - relay_slowdown_trigger_time
    if elapsed < config.RELAY_SLOWDOWN_CHECK_SECONDS:
        return

    required_drop = max(
        config.RELAY_SLOWDOWN_MIN_DROP_GPM,
        relay_slowdown_trigger_flow_gpm * config.RELAY_SLOWDOWN_MIN_DROP_FRACTION,
    )
    actual_drop = relay_slowdown_trigger_flow_gpm - max(0.0, flow_gpm)
    if actual_drop >= required_drop:
        _clear_relay_slowdown_watch("flow_slowed")
        return

    if not relay_slowdown_alarm_active:
        relay_slowdown_alarm_active = True
        log_flow_control(
            "slowdown_alarm"
            f" | elapsed={elapsed:.1f}s"
            f" | trigger_flow_gpm={relay_slowdown_trigger_flow_gpm:.1f}"
            f" | current_flow_gpm={flow_gpm:.1f}"
            f" | drop_gpm={actual_drop:.1f}"
            f" | required_drop_gpm={required_drop:.1f}"
        )


def _record_flow_control_audit(loop_dt_ms, read_ok=True):
    """Track flow-meter sample freshness without adding another IO-Link reader."""
    global flow_control_audit_started_at, flow_control_audit_polls
    global flow_control_audit_fresh, flow_control_audit_duplicates
    global flow_control_audit_flowing_polls, flow_control_audit_flowing_fresh
    global flow_control_audit_errors, flow_control_audit_loop_ms_total
    global flow_control_audit_loop_ms_max

    now = time.time()
    flow_control_audit_polls += 1
    flow_control_audit_loop_ms_total += loop_dt_ms
    flow_control_audit_loop_ms_max = max(flow_control_audit_loop_ms_max, loop_dt_ms)

    if read_ok:
        is_fresh = bool(last_flow_read_was_fresh)
        is_flowing = last_flow_rate >= config.FLOW_STOPPED_THRESHOLD
        if is_fresh:
            flow_control_audit_fresh += 1
        else:
            flow_control_audit_duplicates += 1
        if is_flowing:
            flow_control_audit_flowing_polls += 1
            if is_fresh:
                flow_control_audit_flowing_fresh += 1
    else:
        flow_control_audit_errors += 1

    elapsed = now - flow_control_audit_started_at
    audit_interval = max(1.0, float(getattr(config, "FLOW_CONTROL_AUDIT_INTERVAL", 5.0)))
    if elapsed < audit_interval or flow_control_audit_polls <= 0:
        return

    polls = flow_control_audit_polls
    fresh = flow_control_audit_fresh
    duplicates = flow_control_audit_duplicates
    flowing_polls = flow_control_audit_flowing_polls
    flowing_fresh = flow_control_audit_flowing_fresh
    flowing_duplicates = max(0, flowing_polls - flowing_fresh)
    errors = flow_control_audit_errors
    avg_loop_ms = flow_control_audit_loop_ms_total / polls
    poll_hz = polls / elapsed
    fresh_hz = fresh / elapsed
    flowing_fresh_hz = flowing_fresh / elapsed

    log_flow_control(
        "audit"
        f" | window={elapsed:.1f}s"
        f" | polls={polls}"
        f" | fresh={fresh}"
        f" | dup={duplicates}"
        f" | errors={errors}"
        f" | poll_hz={poll_hz:.1f}"
        f" | fresh_hz={fresh_hz:.1f}"
        f" | flowing_polls={flowing_polls}"
        f" | flowing_fresh={flowing_fresh}"
        f" | flowing_dup={flowing_duplicates}"
        f" | flowing_fresh_hz={flowing_fresh_hz:.1f}"
        f" | loop_avg_ms={avg_loop_ms:.1f}"
        f" | loop_max_ms={flow_control_audit_loop_ms_max:.1f}"
    )

    flow_control_audit_started_at = now
    flow_control_audit_polls = 0
    flow_control_audit_fresh = 0
    flow_control_audit_duplicates = 0
    flow_control_audit_flowing_polls = 0
    flow_control_audit_flowing_fresh = 0
    flow_control_audit_errors = 0
    flow_control_audit_loop_ms_total = 0.0
    flow_control_audit_loop_ms_max = 0.0


def _reset_flow_control_audit(started_at=None):
    """Start a fresh flow-meter freshness audit window."""
    global flow_control_audit_started_at, flow_control_audit_polls
    global flow_control_audit_fresh, flow_control_audit_duplicates
    global flow_control_audit_flowing_polls, flow_control_audit_flowing_fresh
    global flow_control_audit_errors, flow_control_audit_loop_ms_total
    global flow_control_audit_loop_ms_max

    flow_control_audit_started_at = started_at if started_at is not None else time.time()
    flow_control_audit_polls = 0
    flow_control_audit_fresh = 0
    flow_control_audit_duplicates = 0
    flow_control_audit_flowing_polls = 0
    flow_control_audit_flowing_fresh = 0
    flow_control_audit_errors = 0
    flow_control_audit_loop_ms_total = 0.0
    flow_control_audit_loop_ms_max = 0.0


def flow_control_loop():
    """Own IO-Link flow reads and auto-shutoff timing outside the Tk loop."""
    global flow_control_last_tick, flow_control_last_loop_time, flow_control_last_error_log_time

    interval = max(0.01, float(getattr(config, "FLOW_CONTROL_INTERVAL", 0.05)))
    next_tick = time.monotonic()
    flow_control_last_tick = next_tick
    flow_control_last_loop_time = time.time()
    _reset_flow_control_audit(flow_control_last_loop_time)
    log_flow_control(f"thread_start | interval={interval:.3f}s")

    while not flow_control_stop_event.is_set():
        now_mono = time.monotonic()
        loop_dt_ms = (now_mono - flow_control_last_tick) * 1000.0
        flow_control_last_tick = now_mono

        try:
            actual = read_flow_meter()
            update_flow_meter_fault_hold()
            flow_control_last_loop_time = time.time()
            _record_flow_control_audit(loop_dt_ms, read_ok=True)
            _flow_control_process_sample(actual, time.time(), loop_dt_ms)
        except Exception as exc:
            set_pump_stop_fault_hold(True, f"flow control read exception: {exc}")
            flow_control_last_loop_time = time.time()
            _record_flow_control_audit(loop_dt_ms, read_ok=False)
            now = time.time()
            if now - flow_control_last_error_log_time > 5:
                flow_control_last_error_log_time = now
                log_flow_control(f"thread_error | {exc}")

        _maybe_revive_display_chain()

        next_tick += interval
        sleep_for = next_tick - time.monotonic()
        if sleep_for < 0:
            next_tick = time.monotonic()
            sleep_for = 0
        flow_control_stop_event.wait(sleep_for)

    log_flow_control("thread_stop")


def start_flow_control_thread():
    """Start the dedicated flow-control loop when configured."""
    global flow_control_thread

    if not flow_control_enabled():
        log_flow_control("thread_disabled")
        return
    if flow_control_thread and flow_control_thread.is_alive():
        return

    flow_control_stop_event.clear()
    flow_control_thread = threading.Thread(
        target=flow_control_loop,
        name="flow-control",
        daemon=True,
    )
    flow_control_thread.start()



def _wifi_status_snapshot():
    """Return current WiFi status (excluding secrets)."""
    try:
        active_ssid = ''
        ip_addr = ''

        r = subprocess.run(
            ['nmcli', '-t', '-f', 'ACTIVE,SSID,DEVICE', 'dev', 'wifi'],
            capture_output=True,
            text=True,
            timeout=4,
        )
        if r.returncode == 0:
            for line in r.stdout.splitlines():
                parts = line.split(':')
                if len(parts) >= 3 and parts[0] == 'yes':
                    active_ssid = parts[1]
                    break

        r2 = subprocess.run(
            ['nmcli', '-t', '-f', 'IP4.ADDRESS', 'dev', 'show', 'wlan0'],
            capture_output=True,
            text=True,
            timeout=4,
        )
        if r2.returncode == 0:
            for line in r2.stdout.splitlines():
                if line.startswith('IP4.ADDRESS'):
                    ip_addr = line.split(':', 1)[1].split('/')[0].strip()
                    break

        return {
            'ok': bool(active_ssid),
            'connected': bool(active_ssid),
            'ssid': active_ssid,
            'ip': ip_addr,
        }
    except Exception as e:
        return {'ok': False, 'connected': False, 'error': str(e)}


def _wifi_connect(ssid, password, hidden=False):
    """Connect to WiFi using nmcli without logging secrets."""
    ssid = str(ssid or '').strip()
    password = str(password or '')

    if not ssid:
        return {'ok': False, 'code': 'INVALID_SSID', 'message': 'Missing ssid'}
    if len(ssid) > 64:
        return {'ok': False, 'code': 'INVALID_SSID', 'message': 'SSID too long'}
    if len(password) > 128:
        return {'ok': False, 'code': 'INVALID_PASSWORD', 'message': 'Password too long'}

    try:
        # Remove stale connection profile for this SSID to ensure fresh credentials.
        subprocess.run(
            ['nmcli', 'connection', 'delete', ssid],
            capture_output=True,
            text=True,
            timeout=6,
        )

        cmd = ['nmcli', '--wait', '20', 'dev', 'wifi', 'connect', ssid]
        if password:
            cmd += ['password', password]
        if hidden:
            cmd += ['hidden', 'yes']

        r = subprocess.run(cmd, capture_output=True, text=True, timeout=25)
        out = (r.stdout or '') + '\n' + (r.stderr or '')

        if r.returncode != 0:
            low = out.lower()
            if 'secrets were required' in low or 'wrong password' in low or '802-11-wireless-security.key-mgmt' in low:
                return {'ok': False, 'code': 'AUTH_FAILED', 'message': 'Authentication failed'}
            if 'no network with ssid' in low or 'not found' in low:
                return {'ok': False, 'code': 'NO_AP_FOUND', 'message': 'SSID not found'}
            if 'timeout' in low:
                return {'ok': False, 'code': 'TIMEOUT', 'message': 'Connection timeout'}
            return {'ok': False, 'code': 'NMCLI_ERROR', 'message': out.strip()[:160]}

        status = _wifi_status_snapshot()
        status.update({'ok': True, 'code': 'OK'})
        return status

    except subprocess.TimeoutExpired:
        return {'ok': False, 'code': 'TIMEOUT', 'message': 'Connection timeout'}
    except Exception as e:
        return {'ok': False, 'code': 'NMCLI_ERROR', 'message': str(e)}


# nmcli work (status ~8s worst case, connect up to ~31s) must never run
# inline on the serial :9999 listener -- a pump command from another client
# would queue behind it. Status is cached w/ bounded wait; connect runs on a
# background thread and reports via WIFI_STATUS polls.
_wifi_async = AsyncWifiControl(_wifi_status_snapshot, _wifi_connect)


def _wifi_request_validation_error(ssid, password):
    """Fast, radio-free validation (mirrors _wifi_connect's own checks) so
    obviously-bad requests still fail synchronously like they always did."""
    if not ssid:
        return {'code': 'INVALID_SSID', 'message': 'Missing ssid'}
    if len(ssid) > 64:
        return {'code': 'INVALID_SSID', 'message': 'SSID too long'}
    if len(password) > 128:
        return {'code': 'INVALID_PASSWORD', 'message': 'Password too long'}
    return None


def _mouse_int(value, minimum, maximum, default=0):
    try:
        value = int(value)
    except Exception:
        return default
    return max(minimum, min(maximum, value))


def _xdotool_env():
    env = os.environ.copy()
    env.setdefault("DISPLAY", ":0")
    env.setdefault("XDG_RUNTIME_DIR", f"/run/user/{os.getuid()}")
    if not env.get("XAUTHORITY"):
        runtime_dir = Path(env["XDG_RUNTIME_DIR"])
        auth_files = sorted(runtime_dir.glob(".mutter-Xwaylandauth.*"), key=lambda p: p.stat().st_mtime, reverse=True)
        if auth_files:
            env["XAUTHORITY"] = str(auth_files[0])
        else:
            env["XAUTHORITY"] = "/home/pi/.Xauthority"
    return env


def _run_xdotool(args):
    xdotool = shutil.which("xdotool") or "/usr/bin/xdotool"
    if not os.path.exists(xdotool):
        return False, "xdotool not installed"
    try:
        result = subprocess.run(
            [xdotool] + args,
            capture_output=True,
            text=True,
            timeout=1.5,
            env=_xdotool_env(),
        )
        if result.returncode != 0:
            return False, ((result.stderr or result.stdout or "xdotool failed").strip()[:160])
        return True, (result.stdout or "ok").strip()
    except Exception as e:
        return False, str(e)[:160]


def _run_ydotool(args):
    ydotool = shutil.which("ydotool") or "/usr/bin/ydotool"
    if not os.path.exists(ydotool):
        return False, "ydotool not installed"
    try:
        result = subprocess.run(
            [ydotool] + args,
            capture_output=True,
            text=True,
            timeout=1.5,
        )
        if result.returncode != 0:
            return False, ((result.stderr or result.stdout or "ydotool failed").strip()[:160])
        return True, "ok"
    except Exception as e:
        return False, str(e)[:160]


class UInputMouse:
    UI_DEV_CREATE = 0x5501
    UI_DEV_DESTROY = 0x5502
    UI_SET_EVBIT = 0x40045564
    UI_SET_KEYBIT = 0x40045565
    UI_SET_RELBIT = 0x40045566

    EV_SYN = 0
    EV_KEY = 1
    EV_REL = 2
    SYN_REPORT = 0
    REL_X = 0
    REL_Y = 1
    REL_WHEEL = 8
    BTN_LEFT = 0x110
    BTN_RIGHT = 0x111
    BTN_MIDDLE = 0x112
    BUS_USB = 0x03

    def __init__(self):
        self.fd = None
        self.lock = threading.Lock()
        self.available = False
        self.error = ""

    def ensure(self):
        if self.available:
            return True, "ok"
        with self.lock:
            if self.available:
                return True, "ok"
            try:
                self.fd = os.open("/dev/uinput", os.O_WRONLY | os.O_NONBLOCK)
                for evbit in (self.EV_KEY, self.EV_REL):
                    fcntl.ioctl(self.fd, self.UI_SET_EVBIT, evbit)
                for keybit in (self.BTN_LEFT, self.BTN_RIGHT, self.BTN_MIDDLE):
                    fcntl.ioctl(self.fd, self.UI_SET_KEYBIT, keybit)
                for relbit in (self.REL_X, self.REL_Y, self.REL_WHEEL):
                    fcntl.ioctl(self.fd, self.UI_SET_RELBIT, relbit)

                name = b"TrailerSync app cursor"
                user_dev = struct.pack(
                    "80sHHHHi" + ("i" * 256),
                    name,
                    self.BUS_USB,
                    0x5253,
                    0x5453,
                    1,
                    0,
                    *([0] * 256),
                )
                os.write(self.fd, user_dev)
                fcntl.ioctl(self.fd, self.UI_DEV_CREATE)
                time.sleep(0.25)
                self.available = True
                self.error = ""
                return True, "ok"
            except Exception as e:
                self.error = str(e)[:160]
                self.close()
                return False, self.error

    def close(self):
        if self.fd is None:
            self.available = False
            return
        try:
            fcntl.ioctl(self.fd, self.UI_DEV_DESTROY)
        except Exception:
            pass
        try:
            os.close(self.fd)
        except Exception:
            pass
        self.fd = None
        self.available = False

    def emit(self, event_type, code, value):
        now = time.time()
        sec = int(now)
        usec = int((now - sec) * 1_000_000)
        os.write(self.fd, struct.pack("llHHi", sec, usec, event_type, code, int(value)))

    def sync(self):
        self.emit(self.EV_SYN, self.SYN_REPORT, 0)

    def move(self, dx, dy):
        ok, message = self.ensure()
        if not ok:
            return False, message
        with self.lock:
            try:
                if dx:
                    self.emit(self.EV_REL, self.REL_X, dx)
                if dy:
                    self.emit(self.EV_REL, self.REL_Y, dy)
                self.sync()
                return True, "ok"
            except Exception as e:
                self.error = str(e)[:160]
                self.close()
                return False, self.error

    def click(self, button):
        ok, message = self.ensure()
        if not ok:
            return False, message
        button_code = {
            1: self.BTN_LEFT,
            2: self.BTN_MIDDLE,
            3: self.BTN_RIGHT,
        }.get(button, self.BTN_LEFT)
        with self.lock:
            try:
                self.emit(self.EV_KEY, button_code, 1)
                self.sync()
                self.emit(self.EV_KEY, button_code, 0)
                self.sync()
                return True, "ok"
            except Exception as e:
                self.error = str(e)[:160]
                self.close()
                return False, self.error

    def scroll(self, steps):
        ok, message = self.ensure()
        if not ok:
            return False, message
        with self.lock:
            try:
                self.emit(self.EV_REL, self.REL_WHEEL, steps)
                self.sync()
                return True, "ok"
            except Exception as e:
                self.error = str(e)[:160]
                self.close()
                return False, self.error


virtual_mouse = UInputMouse()
atexit.register(virtual_mouse.close)


def _screen_geometry():
    ok, message = _run_xdotool(["getdisplaygeometry"])
    if not ok:
        return 1920, 1080
    parts = str(message).replace("\n", " ").split()
    try:
        return max(1, int(parts[0])), max(1, int(parts[1]))
    except Exception:
        return 1920, 1080


def _pointer_location():
    ok, message = _run_xdotool(["getmouselocation"])
    if not ok:
        return 0, 0
    x = 0
    y = 0
    for part in str(message).split():
        if part.startswith("x:"):
            try:
                x = int(part.split(":", 1)[1])
            except Exception:
                pass
        elif part.startswith("y:"):
            try:
                y = int(part.split(":", 1)[1])
            except Exception:
                pass
    return x, y


def _move_pointer_relative(dx, dy):
    ok, message = virtual_mouse.move(dx, dy)
    if ok:
        return ok, message
    ok, message = _run_xdotool(["mousemove_relative", "--", str(dx), str(dy)])
    if ok:
        return ok, message
    x, y = _pointer_location()
    width, height = _screen_geometry()
    target_x = max(0, min(width - 1, x + dx))
    target_y = max(0, min(height - 1, y + dy))
    return _run_ydotool(["mousemove", "--delay", "0", str(target_x), str(target_y)])


def _ensure_visible_cursor():
    global last_visible_cursor_check
    now = time.time()
    if now - last_visible_cursor_check < 30:
        return
    last_visible_cursor_check = now

    env = os.environ.copy()
    env.setdefault("DBUS_SESSION_BUS_ADDRESS", f"unix:path=/run/user/{os.getuid()}/bus")
    try:
        current_size = subprocess.run(
            ["gsettings", "get", "org.gnome.desktop.interface", "cursor-size"],
            capture_output=True,
            text=True,
            timeout=1.0,
            env=env,
        )
        if (current_size.stdout or "").strip() in ("0", ""):
            subprocess.run(
                ["gsettings", "set", "org.gnome.desktop.interface", "cursor-size", "32"],
                capture_output=True,
                text=True,
                timeout=1.0,
                env=env,
            )
        current_theme = subprocess.run(
            ["gsettings", "get", "org.gnome.desktop.interface", "cursor-theme"],
            capture_output=True,
            text=True,
            timeout=1.0,
            env=env,
        )
        if (current_theme.stdout or "").strip() in ("''", ""):
            subprocess.run(
                ["gsettings", "set", "org.gnome.desktop.interface", "cursor-theme", "Yaru"],
                capture_output=True,
                text=True,
                timeout=1.0,
                env=env,
            )
    except Exception:
        pass


def handle_mouse_command(payload):
    try:
        req = json.loads(payload)
        if not isinstance(req, dict):
            return False, "payload must be object"
    except Exception as e:
        return False, f"invalid json: {e}"

    action = str(req.get("action", "")).strip().lower()
    _ensure_visible_cursor()

    if action == "move":
        dx = _mouse_int(req.get("dx"), -250, 250)
        dy = _mouse_int(req.get("dy"), -250, 250)
        if not dx and not dy:
            return True, "noop"
        return _move_pointer_relative(dx, dy)

    if action == "click":
        button = _mouse_int(req.get("button"), 1, 3, default=1)
        ok, message = virtual_mouse.click(button)
        if ok:
            return ok, message
        ok, message = _run_xdotool(["click", str(button)])
        if ok:
            return ok, message
        ydotool_button = 2 if button == 3 else (3 if button == 2 else button)
        return _run_ydotool(["click", "--delay", "0", str(ydotool_button)])

    if action == "scroll":
        steps = _mouse_int(req.get("steps"), -8, 8)
        if not steps:
            return True, "noop"
        ok, message = virtual_mouse.scroll(steps)
        if ok:
            return ok, message
        button = "4" if steps > 0 else "5"
        ok = True
        message = "ok"
        for _ in range(abs(steps)):
            ok, message = _run_xdotool(["click", button])
            if not ok:
                ok, message = _run_ydotool(["click", "--delay", "0", button])
            if not ok:
                break
        return ok, message

    if action == "key":
        key = str(req.get("key", "")).strip().lower()
        key_map = {
            "esc": "Escape",
            "escape": "Escape",
            "enter": "Return",
            "return": "Return",
            "alt_f4": "alt+F4",
        }
        mapped = key_map.get(key)
        if not mapped:
            return False, f"unsupported key: {key}"
        ok, message = _run_ydotool(["key", "--delay", "0", mapped])
        if ok:
            return ok, message
        return _run_xdotool(["key", mapped])

    return False, f"unsupported action: {action}"


def _run_on_tk_queue_and_wait(callback, timeout_seconds=1.5):
    """Run a short state mutation on Tk's queue before acknowledging it.

    Sensor updates are queued through ``root.after``.  Trailer identity
    commands must use that same queue so an older sensor callback cannot run
    after a reset has already been acknowledged.  The bounded wait also keeps
    the socket listener from hanging when the Tk loop is shutting down.
    """
    completed = threading.Event()
    state_lock = threading.Lock()
    state = {"cancelled": False, "error": None}

    def queued_callback():
        with state_lock:
            if state["cancelled"]:
                completed.set()
                return
            try:
                callback()
            except Exception as exc:
                state["error"] = exc
            finally:
                completed.set()

    try:
        root.after(0, queued_callback)
    except Exception as exc:
        print(f"Tk command barrier could not be queued: {exc}", flush=True)
        return False

    if not completed.wait(max(0.0, float(timeout_seconds))):
        # If the callback has not started, cancel it while holding the same
        # lock it must acquire.  This prevents a timed-out reset from mutating
        # state later after RotorLink has treated the command as failed.
        with state_lock:
            if not completed.is_set():
                state["cancelled"] = True
                print("Tk command barrier timed out", flush=True)
                return False

    if state["error"] is not None:
        print(f"Tk command barrier failed: {state['error']}", flush=True)
        return False
    return True


def _reset_trailer_sensor_telemetry_and_refresh():
    """Reset trailer telemetry and redraw it on the Tk thread."""
    _reset_trailer_sensor_telemetry()
    update_mopeka_display()
    update_bms_display()


def _run_trailer_sensor_identity_command(line, timeout_seconds=1.5):
    """Serialize a recognized trailer identity command; return None otherwise."""
    if line == "RESET_TRAILER_SENSOR_TELEMETRY":
        callback = _reset_trailer_sensor_telemetry_and_refresh
    elif line == "TRAILER_SENSOR_IDENTITY_CHANGED":
        callback = _mark_trailer_sensor_identity_changed
    else:
        return None
    return _run_on_tk_queue_and_wait(callback, timeout_seconds=timeout_seconds)


def socket_command_listener():
    """Listen for commands from rotorsync BLE server via localhost socket"""
    global requested_gallons, override_mode, override_enabled_time, colors_are_green
    global fill_requested_gallons, mix_requested_gallons, current_mode, batch_mix_data

    import socket as sock_module

    DASHBOARD_PORT = 9999
    debug_log = config.SERIAL_DEBUG_LOG

    sock_server = sock_module.socket(sock_module.AF_INET, sock_module.SOCK_STREAM)
    sock_server.setsockopt(sock_module.SOL_SOCKET, sock_module.SO_REUSEADDR, 1)
    sock_server.bind(("127.0.0.1", DASHBOARD_PORT))
    sock_server.listen(8)
    sock_server.settimeout(1.0)

    print(f"Socket listener started on port {DASHBOARD_PORT}")
    append_debug_log(debug_log, f"{time.strftime('%Y-%m-%d %H:%M:%S')} - Socket listener started on port {DASHBOARD_PORT}\n")

    while True:
        try:
            try:
                client, addr = sock_server.accept()
                client.settimeout(5.0)
                try:
                    # Commands are newline-terminated `<cmd>\n` (see
                    # rotorlink/dashboard_client). Read until the newline rather
                    # than a single recv(4096): a long payload (e.g. a multi-product
                    # BATCHMIX:{...}) exceeds 4096 and was truncated into invalid
                    # JSON and silently dropped, leaving a stale mix target. Decode
                    # tolerantly so a multibyte char split across a recv boundary
                    # can't raise UnicodeDecodeError.
                    raw = b""
                    while b"\n" not in raw and len(raw) < 65536:
                        chunk = client.recv(4096)
                        if not chunk:
                            break
                        raw += chunk
                    data = raw.decode("utf-8", "replace").strip()
                    if data:
                        for line in data.split("\n"):
                            line = line.strip()
                            if not line:
                                continue

                            safe_line = line
                            if line.startswith('WIFI_SET:'):
                                try:
                                    payload = line[9:]
                                    req = json.loads(payload)
                                    if isinstance(req, dict) and 'password' in req:
                                        req['password'] = '***'
                                    safe_line = f"WIFI_SET:{json.dumps(req, separators=(',', ':'))}"
                                except Exception:
                                    safe_line = 'WIFI_SET:{...}'

                            if line != "STATE_JSON":
                                append_debug_log(debug_log, f"{time.strftime('%Y-%m-%d %H:%M:%S')} - Socket received: '{safe_line}'\n")

                            if line == "STATUS":
                                actual = last_totalizer_liters * config.LITERS_TO_GALLONS
                                response = f"REQ:{requested_gallons:.1f}|ACT:{actual:.1f}|MODE:{current_mode}\n"
                                client.send(response.encode())
                                continue

                            if line == "STATE_JSON":
                                snapshot = _build_dashboard_state_snapshot()
                                client.send(
                                    f"STATE_JSON:{json.dumps(snapshot, separators=(',', ':'))}\n".encode()
                                )
                                continue

                            if line == "LIVE_TELEMETRY":
                                actual = last_totalizer_liters * config.LITERS_TO_GALLONS
                                flow_gpm = last_flow_rate * config.LITERS_PER_SEC_TO_GPM
                                flow_fault_active, flow_fault_code, flow_fault_reason = _flow_meter_fault_summary()
                                payload = {
                                    "req": round(requested_gallons, 3),
                                    "act": round(actual, 3),
                                    "flow": round(flow_gpm, 2),
                                    "rs": bool(relay_slowdown_alarm_active),
                                    "ff": bool(flow_fault_active),
                                }
                                if flow_fault_active:
                                    payload["fc"] = flow_fault_code
                                    if flow_fault_reason:
                                        payload["fmr"] = flow_fault_reason
                                client.send(
                                    f"LIVE:{json.dumps(payload, separators=(',', ':'))}\n".encode()
                                )
                                continue

                            if line in (
                                "RESET_TRAILER_SENSOR_TELEMETRY",
                                "TRAILER_SENSOR_IDENTITY_CHANGED",
                            ):
                                # Both mutations share the ordered Tk queue
                                # with sensor applies.  OK means the command has
                                # run; ERROR makes RotorLink abort/roll back.
                                command_applied = _run_trailer_sensor_identity_command(line)
                                client.send(b"OK\n" if command_applied else b"ERROR\n")
                                continue

                            elif line == "ACCEPT_PENDING_CURVE":
                                ok, payload = accept_pending_flow_curve("Socket")
                                prefix = "CURVE_ACCEPTED" if ok else "CURVE_ACCEPT_ERR"
                                client.send(
                                    f"{prefix}:{json.dumps(payload, separators=(',', ':'))}\n".encode()
                                )
                                continue

                            elif line == "MIX":
                                root.after(0, lambda: switch_mode("mix"))

                            elif line == "RESET":
                                root.after(0, lambda: force_flow_reset("socket_reset"))

                            elif line.startswith("CAL_START:"):
                                try:
                                    cal_params = json.loads(line[len("CAL_START:"):])
                                except Exception as ce:
                                    client.send(f"CAL_ERR:invalid params: {ce}\n".encode())
                                    continue
                                ok, cal_err = start_tank_calibration_remote(cal_params)
                                client.send((f"CAL_OK\n" if ok else f"CAL_ERR:{cal_err}\n").encode())
                                continue

                            elif line == "CAL_CONFIRM":
                                if calibration_mode:
                                    root.after(0, calibration_confirm)
                                    client.send(b"CAL_OK\n")
                                else:
                                    client.send(b"CAL_ERR:not running\n")
                                continue

                            elif line == "CAL_CANCEL":
                                if calibration_mode:
                                    root.after(0, calibration_cancel)
                                    client.send(b"CAL_OK\n")
                                else:
                                    client.send(b"CAL_ERR:not running\n")
                                continue

                            elif line.startswith("CAL_ADJUST:"):
                                try:
                                    cal_delta = int(line[len("CAL_ADJUST:"):])
                                except ValueError:
                                    client.send(b"CAL_ERR:invalid delta\n")
                                    continue
                                if calibration_mode:
                                    root.after(0, lambda d=cal_delta: calibration_adjust_value(d))
                                    client.send(b"CAL_OK\n")
                                else:
                                    client.send(b"CAL_ERR:not running\n")
                                continue

                            elif line == "TU":
                                root.after(0, lambda: handle_thumbs_up_press("socket TU"))

                            elif line.startswith("PILOT_CONNECTED:"):
                                pilot_name = line[len("PILOT_CONNECTED:"):].strip()
                                root.after(0, lambda n=pilot_name: update_pilot_status(True, n))
                                client.send(b"PILOT_OK\n")
                                continue

                            elif line.startswith("PILOT_DISCONNECTED:"):
                                pilot_name = line[len("PILOT_DISCONNECTED:"):].strip()
                                root.after(0, lambda n=pilot_name: update_pilot_status(False, n))
                                client.send(b"PILOT_OK\n")
                                continue

                            elif line.startswith("WIFI_PILOT_CONNECTED:"):
                                pilot_name = line[len("WIFI_PILOT_CONNECTED:"):].strip()
                                root.after(0, lambda n=pilot_name: update_wifi_pilot_status(True, n))
                                client.send(b"PILOT_OK\n")
                                continue

                            elif line.startswith("WIFI_PILOT_DISCONNECTED:"):
                                pilot_name = line[len("WIFI_PILOT_DISCONNECTED:"):].strip()
                                root.after(0, lambda n=pilot_name: update_wifi_pilot_status(False, n))
                                client.send(b"PILOT_OK\n")
                                continue

                            elif line.startswith("PILOT_LOC:"):
                                loc_payload = line[len("PILOT_LOC:"):].strip()
                                root.after(0, lambda p=loc_payload: update_pilot_loc("ble", p))
                                client.send(b"LOC_OK\n")
                                continue

                            elif line.startswith("WIFI_PILOT_LOC:"):
                                loc_payload = line[len("WIFI_PILOT_LOC:"):].strip()
                                root.after(0, lambda p=loc_payload: update_pilot_loc("wifi", p))
                                client.send(b"LOC_OK\n")
                                continue

                            elif line == "FILL":
                                root.after(0, lambda: switch_mode("fill"))

                            elif line == "RUN_UPDATE":
                                msg = "Socket: Software update command received"
                                print(msg)
                                append_debug_log(debug_log, f"{time.strftime('%Y-%m-%d %H:%M:%S')} - {msg}\n")
                                if update_mode:
                                    client.send(b"UPDATE_ALREADY_RUNNING\n")
                                else:
                                    # Same entry point as the box menu's SYSTEM
                                    # UPDATE — fullscreen progress on the box
                                    # screen, services restart at the end.
                                    root.after(0, run_system_update)
                                    client.send(b"UPDATE_STARTED\n")
                                continue

                            elif line == "REBOOT":
                                msg = "Socket: Reboot command received"
                                print(msg)
                                append_debug_log(debug_log, f"{time.strftime('%Y-%m-%d %H:%M:%S')} - {msg}\n")
                                root.after(0, reboot_system)

                            elif line == "SHUTDOWN":
                                msg = "Socket: Shutdown command received"
                                print(msg)
                                append_debug_log(debug_log, f"{time.strftime('%Y-%m-%d %H:%M:%S')} - {msg}\n")
                                root.after(0, shutdown_system)

                            elif line.startswith("WIFI_SET:"):
                                try:
                                    payload = line[9:]
                                    req = json.loads(payload)
                                    ssid = req.get('ssid', '')
                                    password = req.get('password', '')
                                    hidden = bool(req.get('hidden', False))

                                    # Log without secrets
                                    msg = f"Socket: WIFI_SET requested for SSID '{ssid}' (hidden={hidden})"
                                    print(msg)
                                    append_debug_log(debug_log, f"{time.strftime('%Y-%m-%d %H:%M:%S')} - {msg}\n")

                                    validation_error = _wifi_request_validation_error(
                                        str(ssid or '').strip(), str(password or '')
                                    )
                                    if validation_error is not None:
                                        client.send(f"WIFI_ERR:{json.dumps(validation_error, separators=(',', ':'))}\n".encode())
                                        continue
                                    result = _wifi_async.request_connect(ssid, password, hidden)
                                    if result.get('ok'):
                                        client.send(f"WIFI_OK:{json.dumps(result, separators=(',', ':'))}\n".encode())
                                    else:
                                        err_payload = {
                                            'code': result.get('code', 'NMCLI_ERROR'),
                                            'message': result.get('message', ''),
                                        }
                                        client.send(f"WIFI_ERR:{json.dumps(err_payload, separators=(',', ':'))}\n".encode())
                                    continue
                                except Exception as we:
                                    err_payload = {'code': 'NMCLI_ERROR', 'message': str(we)}
                                    client.send(f"WIFI_ERR:{json.dumps(err_payload, separators=(',', ':'))}\n".encode())
                                    continue

                            elif line == "WIFI_STATUS":
                                status = _wifi_async.status()
                                client.send(f"WIFI_STATUS:{json.dumps(status, separators=(',', ':'))}\n".encode())
                                continue

                            elif line.startswith("MOUSE:"):
                                payload = line[6:]
                                ok, mouse_message = handle_mouse_command(payload)
                                if ok:
                                    client.send(b"MOUSE_OK\n")
                                else:
                                    client.send(f"MOUSE_ERR:{mouse_message}\n".encode())
                                continue

                            elif line.startswith("BATCHMIX_ERROR:"):
                                # Handle BatchMix validation error
                                error_msg = line[15:]
                                msg = f"Socket: BatchMix ERROR - {error_msg}"
                                print(msg)
                                append_debug_log(debug_log, f"{time.strftime('%Y-%m-%d %H:%M:%S')} - {msg}\n")
                                # Display error on screen
                                root.after(0, lambda e=error_msg: show_batchmix_error(e))

                            elif line.startswith("BATCHMIX:"):
                                try:
                                    json_str = line[9:]
                                    batch_mix_data = json.loads(json_str)
                                    msg = f"Socket: BatchMix received - {len(batch_mix_data.get('products', []))} products"
                                    print(msg)
                                    append_debug_log(debug_log, f"{time.strftime('%Y-%m-%d %H:%M:%S')} - {msg}\n")

                                    # Store the BatchMix target as the mix preset. If we are already in
                                    # mix mode, also update the live target used by auto-stop.
                                    water_needed = batch_mix_data.get('water_needed', 0)
                                    if water_needed > 0:
                                        water_needed = float(water_needed)
                                        mix_requested_gallons = water_needed
                                        save_mode_presets()
                                        # Only update display if in mix mode
                                        if current_mode == "mix":
                                            requested_gallons = water_needed
                                            colors_are_green = False
                                            # Show decimal if present, otherwise whole number
                                            if water_needed == int(water_needed):
                                                req_str = f"{int(water_needed)}"
                                            else:
                                                req_str = f"{water_needed:.1f}"
                                            root.after(0, lambda s=req_str: draw_requested_number(s, "green" if colors_are_green else "red"))

                                    root.after(0, update_batch_mix_overlay)
                                except Exception as bme:
                                    msg = f"Socket: BatchMix parse error: {bme}"
                                    print(msg)
                                    append_debug_log(debug_log, f"{time.strftime('%Y-%m-%d %H:%M:%S')} - {msg}\n")


                            elif line == "MOPEKA_OFFLINE":
                                root.after(0, _mopeka_offline)

                            elif line == "MOPEKA_DISABLED":
                                root.after(0, _mopeka_disabled)

                            elif line.startswith("MOPEKA_SENSOR:"):
                                try:
                                    parsed = _parse_mopeka_sensor_command(line)
                                    if parsed is not None:
                                        root.after(
                                            0,
                                            _apply_mopeka_sensor,
                                            *parsed,
                                        )
                                except Exception as me:
                                    print(f"Mopeka sensor parse error: {me}", flush=True)

                            elif line.startswith("MOPEKA:"):
                                try:
                                    parts = line[7:].split("|")
                                    if len(parts) >= 4:
                                        _m1g = float(parts[0])
                                        _m2g = float(parts[1])
                                        _m1q = int(parts[2])
                                        _m2q = int(parts[3])
                                        _m1ts = parts[4] if len(parts) > 4 else None
                                        _m2ts = parts[5] if len(parts) > 5 else None
                                        _identity = parts[6] if len(parts) > 6 else None
                                        root.after(
                                            0,
                                            _apply_mopeka,
                                            _m1g,
                                            _m2g,
                                            _m1q,
                                            _m2q,
                                            _m1ts,
                                            _m2ts,
                                            _identity,
                                        )
                                except Exception as me:
                                    print(f"Mopeka parse error: {me}", flush=True)

                            elif line.startswith("MOPEKA_RAW:"):
                                try:
                                    parts = line[11:].split("|")
                                    if len(parts) >= 4:
                                        root.after(
                                            0,
                                            _apply_mopeka_raw,
                                            float(parts[0]),
                                            float(parts[1]),
                                            float(parts[2]),
                                            float(parts[3]),
                                        )
                                except Exception as me:
                                    print(f"Mopeka raw parse error: {me}", flush=True)

                            elif line.startswith("BMS:"):
                                try:
                                    parts = line[4:].split("|")
                                    if len(parts) >= 2:
                                        observed_at = parts[2] if len(parts) > 2 else None
                                        identity_token = parts[3] if len(parts) > 3 else None
                                        root.after(
                                            0,
                                            _apply_bms,
                                            float(parts[0]),
                                            float(parts[1]),
                                            observed_at,
                                            identity_token,
                                        )
                                except Exception as be:
                                    print(f"BMS parse error: {be}", flush=True)

                            elif line == "HISTORY":
                                try:
                                    with open("/home/pi/fill_history.log", "r") as hf:
                                        all_lines = hf.readlines()
                                        latest_entries = all_lines[-FILL_HISTORY_SOCKET_LIMIT:]
                                        history_items = []
                                        for entry in reversed(latest_entries):
                                            parts = entry.strip().split("|")
                                            if len(parts) >= 3:
                                                ts = parts[0].strip()
                                                req = float(_history_named_field(parts, "Requested").replace("gal", "").strip())
                                                act = float(_history_named_field(parts, "Actual").replace("gal", "").strip())
                                                shutoff_type = ""
                                                for part in parts[3:]:
                                                    text = part.strip()
                                                    if text.lower().startswith(("auto", "manual")):
                                                        shutoff_type = text
                                                        break
                                                temp_token = ""
                                                temp_text = _history_named_field(parts, "Temp").replace("F", "").strip()
                                                if temp_text:
                                                    temp_token = _base36_encode(round(float(temp_text) * 10.0))
                                                stop_to_thumb_token = ""
                                                stop_to_thumb_text = (
                                                    _history_named_field(parts, "StopToThumb")
                                                    .replace("s", "")
                                                    .strip()
                                                )
                                                if stop_to_thumb_text:
                                                    stop_to_thumb_token = _base36_encode(round(float(stop_to_thumb_text) * 10.0))
                                                timestamp = time.mktime(time.strptime(ts, "%Y-%m-%d %H:%M:%S"))
                                                minute_token = _base36_encode(int(timestamp // 60))
                                                req_token = _base36_encode(round(req * 1000.0))
                                                act_token = _base36_encode(round(act * 1000.0))
                                                shutoff_token = "a" if shutoff_type.lower().startswith("auto") else "m"
                                                history_items.append(
                                                    f"{minute_token},{req_token},{act_token},{shutoff_token},{temp_token},{stop_to_thumb_token}"
                                                )
                                        history_response = "H2:" + ";".join(history_items)
                                        client.send(f"HIST:{history_response}\n".encode())
                                except Exception:
                                    client.send(b"HIST:\n")
                                continue

                            elif line.startswith("SET_REQUESTED_GALLONS:"):
                                value = line.split(":", 1)[1].strip()
                                ok, result = set_requested_gallons(value, "Socket")
                                if ok:
                                    client.send(f"SET_REQUESTED_GALLONS_OK:{float(result):.3f}\n".encode())
                                else:
                                    client.send(f"SET_REQUESTED_GALLONS_ERR:{result}\n".encode())
                                continue

                            elif line in ['+1', '-1', '+10', '-10']:
                                if menu_mode:
                                    if line == '+1':
                                        msg = "Socket: Menu navigate down"
                                        print(msg)
                                        append_debug_log(debug_log, f"{time.strftime('%Y-%m-%d %H:%M:%S')} - {msg}\n")
                                        menu_navigate_down()
                                    elif line == '-1':
                                        msg = "Socket: Menu navigate up"
                                        print(msg)
                                        append_debug_log(debug_log, f"{time.strftime('%Y-%m-%d %H:%M:%S')} - {msg}\n")
                                        menu_navigate_up()
                                    else:
                                        append_debug_log(debug_log, f"{time.strftime('%Y-%m-%d %H:%M:%S')} - Ignored in menu mode: '{line}'\n")
                                else:
                                    try:
                                        adjustment = int(line)
                                        if adjust_batch_mix_gallons(adjustment, "Socket"):
                                            continue
                                        requested_gallons += adjustment
                                        if requested_gallons < 0:
                                            requested_gallons = 0
                                        colors_are_green = False
                                        if current_mode == 'fill':
                                            fill_requested_gallons = requested_gallons
                                        else:
                                            mix_requested_gallons = requested_gallons
                                        save_mode_presets()
                                        msg = f"Socket: Adjusted by {adjustment}, requested gallons now {requested_gallons}"
                                        print(msg)
                                        append_debug_log(debug_log, f"{time.strftime('%Y-%m-%d %H:%M:%S')} - {msg}\n")
                                    except ValueError:
                                        pass

                            elif line == 'PS':
                                if exit_confirm_window:
                                    msg = "Socket: Exit confirmation cancel"
                                    print(msg)
                                    append_debug_log(debug_log, f"{time.strftime('%Y-%m-%d %H:%M:%S')} - {msg}\n")
                                    if exit_cancel_handler:
                                        root.after(0, exit_cancel_handler)
                                elif reset_season_confirm_window:
                                    msg = "Socket: Reset season confirmation cancel"
                                    print(msg)
                                    append_debug_log(debug_log, f"{time.strftime('%Y-%m-%d %H:%M:%S')} - {msg}\n")
                                    if reset_season_cancel_handler:
                                        root.after(0, reset_season_cancel_handler)
                                elif reset_flow_curve_confirm_window:
                                    msg = "Socket: Flow curve reset confirmation cancel"
                                    print(msg)
                                    append_debug_log(debug_log, f"{time.strftime('%Y-%m-%d %H:%M:%S')} - {msg}\n")
                                    if reset_flow_curve_cancel_handler:
                                        root.after(0, reset_flow_curve_cancel_handler)
                                elif accept_flow_curve_confirm_window:
                                    msg = "Socket: Flow curve accept confirmation cancel"
                                    print(msg)
                                    append_debug_log(debug_log, f"{time.strftime('%Y-%m-%d %H:%M:%S')} - {msg}\n")
                                    if accept_flow_curve_cancel_handler:
                                        root.after(0, accept_flow_curve_cancel_handler)
                                elif calibration_mode:
                                    msg = "Socket: Calibration cancel/back"
                                    print(msg)
                                    append_debug_log(debug_log, f"{time.strftime('%Y-%m-%d %H:%M:%S')} - {msg}\n")
                                    root.after(0, calibration_cancel)
                                elif log_viewer_mode:
                                    msg = "Socket: Log viewer exit"
                                    print(msg)
                                    append_debug_log(debug_log, f"{time.strftime('%Y-%m-%d %H:%M:%S')} - {msg}\n")
                                    root.after(0, close_log_viewer)
                                elif fill_history_mode:
                                    msg = "Socket: Fill history exit"
                                    print(msg)
                                    append_debug_log(debug_log, f"{time.strftime('%Y-%m-%d %H:%M:%S')} - {msg}\n")
                                    root.after(0, close_fill_history)
                                elif self_test_mode:
                                    msg = "Socket: Self-test exit"
                                    print(msg)
                                    append_debug_log(debug_log, f"{time.strftime('%Y-%m-%d %H:%M:%S')} - {msg}\n")
                                    root.after(0, close_self_test)
                                elif full_test_mode:
                                    msg = "Socket: Full-test PS command detected"
                                    print(msg)
                                    append_debug_log(debug_log, f"{time.strftime('%Y-%m-%d %H:%M:%S')} - {msg}\n")
                                    if full_test_window and hasattr(full_test_window, 'mark_tested'):
                                        root.after(0, lambda: full_test_window.mark_tested('PS'))
                                elif update_mode:
                                    msg = "Socket: Update exit"
                                    print(msg)
                                    append_debug_log(debug_log, f"{time.strftime('%Y-%m-%d %H:%M:%S')} - {msg}\n")
                                    root.after(0, close_update)
                                elif menu_mode:
                                    msg = "Socket: Menu close"
                                    print(msg)
                                    append_debug_log(debug_log, f"{time.strftime('%Y-%m-%d %H:%M:%S')} - {msg}\n")
                                    root.after(0, close_menu)
                                else:
                                    msg = "Socket: Pump Stop command received"
                                    print(msg)
                                    append_debug_log(debug_log, f"{time.strftime('%Y-%m-%d %H:%M:%S')} - {msg}\n")
                                    start_pump_stop_thread(config.PUMP_STOP_DURATION)

                            elif line in ('OV:1', 'OV:0'):
                                override_mode = (line == 'OV:1')
                                if override_mode:
                                    override_enabled_time = time.time()
                                msg = f"Socket: Override mode {'ENABLED' if override_mode else 'DISABLED'} (explicit)"
                                print(msg)
                                append_debug_log(debug_log, f"{time.strftime('%Y-%m-%d %H:%M:%S')} - {msg}\n")

                            elif line == 'OV':
                                if menu_mode:
                                    msg = "Socket: Menu select"
                                    print(msg)
                                    append_debug_log(debug_log, f"{time.strftime('%Y-%m-%d %H:%M:%S')} - {msg}\n")
                                    arm_menu_ov_guard()
                                    root.after(0, menu_select)
                                elif requested_gallons == 0:
                                    if current_mode == 'mix' and batch_mix_data is not None:
                                        msg = "Socket: Batch mix screen exit triggered (gallons=0, OV pressed)"
                                        print(msg)
                                        append_debug_log(debug_log, f"{time.strftime('%Y-%m-%d %H:%M:%S')} - {msg}\n")
                                        root.after(0, lambda: clear_batch_mix_screen("socket OV at zero gallons"))
                                    else:
                                        msg = "Socket: Menu access triggered (gallons=0, OV pressed)"
                                        print(msg)
                                        append_debug_log(debug_log, f"{time.strftime('%Y-%m-%d %H:%M:%S')} - {msg}\n")
                                        arm_menu_ov_guard()
                                        root.after(0, show_menu)
                                else:
                                    override_mode = not override_mode
                                    if override_mode:
                                        override_enabled_time = time.time()
                                    msg = f"Socket: Override mode {'ENABLED' if override_mode else 'DISABLED'}"
                                    print(msg)
                                    append_debug_log(debug_log, f"{time.strftime('%Y-%m-%d %H:%M:%S')} - {msg}\n")

                            client.send(b"OK\n")
                except sock_module.timeout:
                    pass
                finally:
                    client.close()
            except sock_module.timeout:
                pass
        except Exception as e:
            print(f"Socket listener error: {e}")
            time.sleep(1)

def _log_ov_diag(source):
    """One-line context snapshot on every OV, to catch what triggers a phantom OV.
    Event-driven (OV is rare), reads existing globals only, wrapped so it can never
    break the serial loop. idle_since_operator_input is the key signal: large => the
    OV arrived with no recent knob/button activity (a true no-press phantom); small =>
    bounce/mispress. Cross-reference the timestamp against the rotorsync journal
    'GATT client connected' lines for BLE correlation."""
    try:
        idle = (time.time() - last_operator_input_ts) if last_operator_input_ts else -1.0
        msg = (f"[OV-DIAG] {source} idle_since_operator_input={idle:.1f}s "
               f"flow_gpm={flow_gpm:.2f} pump_latched={auto_shutoff_latched} "
               f"slowdown_alarm={relay_slowdown_alarm_active} switch_box={switch_box_connected} "
               f"override_before={override_mode} gallons={requested_gallons} menu={menu_mode}")
        print(msg)
        log_serial_debug(msg)
    except Exception as e:
        try:
            log_serial_debug(f"[OV-DIAG] snapshot error: {e}")
        except Exception:
            pass


def serial_listener():
    """Listen for serial messages with format: requested,actual"""
    global requested_gallons, serial_connected, override_mode, colors_are_green, last_heartbeat_time
    global fill_requested_gallons, mix_requested_gallons, current_mode
    global last_serial_ov_toggle_time, last_operator_input_ts

    debug_log = config.SERIAL_DEBUG_LOG
    buffer = ""

    try:
        ser = serial.Serial(config.SERIAL_PORT, config.SERIAL_BAUD, timeout=0.5)
        ser.reset_input_buffer()
        serial_connected = True
        msg = f"Serial listener started on {config.SERIAL_PORT} at {config.SERIAL_BAUD} baud"
        print(msg)
        log_serial_debug(msg)

        while True:
            try:
                if ser.in_waiting > 0:
                    # Read all available bytes
                    raw_bytes = ser.read(ser.in_waiting)
                    log_serial_debug(f"Raw bytes: {raw_bytes} (hex: {raw_bytes.hex()})")

                    # Decode and add to buffer
                    chunk = raw_bytes.decode('utf-8', errors='ignore')
                    buffer += chunk
                    log_serial_debug(f"Decoded: '{chunk}' | Buffer now: '{buffer}'")

                    # Process complete lines (ending with \n or \r)
                    while '\n' in buffer or '\r' in buffer:
                        # Split on either \n or \r
                        if '\n' in buffer:
                            line, buffer = buffer.split('\n', 1)
                        else:
                            line, buffer = buffer.split('\r', 1)

                        line = line.strip()
                        log_serial_debug(f"Complete line: '{line}'")

                        if line:
                            # Update heartbeat if we receive OK message
                            if line == 'OK':
                                last_heartbeat_time = time.time()
                                log_serial_debug("Heartbeat received (OK)")
                                continue  # Don't process OK as a command

                            # --- Phantom-OV diagnostics (event-driven; OV is rare) ---
                            if line == 'OV':
                                _log_ov_diag("Serial")
                            elif line != 'BOOT':
                                last_operator_input_ts = time.time()
                            # ---------------------------------------------------------

                            # A physical-reset incident must always be escapable,
                            # even if a menu or confirmation screen is open. Handle
                            # Pump Stop synchronously so normal state returns as soon
                            # as the physical button command is received.
                            if line == 'PS' and physical_reset_safety_active:
                                msg = "Serial: Pump Stop acknowledged physical-reset safety"
                                print(msg)
                                append_debug_log(
                                    debug_log,
                                    f"{time.strftime('%Y-%m-%d %H:%M:%S')} - {msg}\n",
                                )
                                handle_box_pump_stop_button("serial pump stop")
                                continue

                            if should_ignore_menu_ov_bounce(line, "Serial"):
                                continue

                            # Handle exit confirmation dialog
                            if exit_confirm_window:
                                if line == 'OV':
                                    msg = "Serial: Exit confirmation (OV - Confirm)"
                                    print(msg)
                                    append_debug_log(debug_log, f"{time.strftime('%Y-%m-%d %H:%M:%S')} - {msg}\n")
                                    if exit_confirm_handler:
                                        root.after(0, exit_confirm_handler)
                                elif line == 'PS':
                                    msg = "Serial: Exit confirmation (PS - Cancel)"
                                    print(msg)
                                    append_debug_log(debug_log, f"{time.strftime('%Y-%m-%d %H:%M:%S')} - {msg}\n")
                                    if exit_cancel_handler:
                                        root.after(0, exit_cancel_handler)
                                else:
                                    append_debug_log(debug_log, f"{time.strftime('%Y-%m-%d %H:%M:%S')} - Ignored in exit confirmation: '{line}'\n")

                            # Handle reset season confirmation dialog
                            elif reset_season_confirm_window:
                                if line == 'OV':
                                    msg = "Serial: Reset season confirmation (OV - Confirm)"
                                    print(msg)
                                    append_debug_log(debug_log, f"{time.strftime('%Y-%m-%d %H:%M:%S')} - {msg}\n")
                                    if reset_season_confirm_handler:
                                        root.after(0, reset_season_confirm_handler)
                                elif line == 'PS':
                                    msg = "Serial: Reset season confirmation (PS - Cancel)"
                                    print(msg)
                                    append_debug_log(debug_log, f"{time.strftime('%Y-%m-%d %H:%M:%S')} - {msg}\n")
                                    if reset_season_cancel_handler:
                                        root.after(0, reset_season_cancel_handler)
                                else:
                                    append_debug_log(debug_log, f"{time.strftime('%Y-%m-%d %H:%M:%S')} - Ignored in reset season confirmation: '{line}'\n")

                            # Handle flow curve reset confirmation dialog
                            elif reset_flow_curve_confirm_window:
                                if line == 'OV':
                                    msg = "Serial: Flow curve reset confirmation (OV - Confirm)"
                                    print(msg)
                                    append_debug_log(debug_log, f"{time.strftime('%Y-%m-%d %H:%M:%S')} - {msg}\n")
                                    if reset_flow_curve_confirm_handler:
                                        root.after(0, reset_flow_curve_confirm_handler)
                                elif line == 'PS':
                                    msg = "Serial: Flow curve reset confirmation (PS - Cancel)"
                                    print(msg)
                                    append_debug_log(debug_log, f"{time.strftime('%Y-%m-%d %H:%M:%S')} - {msg}\n")
                                    if reset_flow_curve_cancel_handler:
                                        root.after(0, reset_flow_curve_cancel_handler)
                                else:
                                    append_debug_log(debug_log, f"{time.strftime('%Y-%m-%d %H:%M:%S')} - Ignored in flow curve reset confirmation: '{line}'\n")

                            # Handle learned flow curve accept confirmation dialog
                            elif accept_flow_curve_confirm_window:
                                if line == 'OV':
                                    msg = "Serial: Flow curve accept confirmation (OV - Confirm)"
                                    print(msg)
                                    append_debug_log(debug_log, f"{time.strftime('%Y-%m-%d %H:%M:%S')} - {msg}\n")
                                    if accept_flow_curve_confirm_handler:
                                        root.after(0, accept_flow_curve_confirm_handler)
                                elif line == 'PS':
                                    msg = "Serial: Flow curve accept confirmation (PS - Cancel)"
                                    print(msg)
                                    append_debug_log(debug_log, f"{time.strftime('%Y-%m-%d %H:%M:%S')} - {msg}\n")
                                    if accept_flow_curve_cancel_handler:
                                        root.after(0, accept_flow_curve_cancel_handler)
                                else:
                                    append_debug_log(debug_log, f"{time.strftime('%Y-%m-%d %H:%M:%S')} - Ignored in flow curve accept confirmation: '{line}'\n")

                            # Handle reminders mode - dismiss on OV
                            elif reminders_mode:
                                if line == 'OV':
                                    msg = "Serial: Dismiss reminders"
                                    print(msg)
                                    append_debug_log(debug_log, f"{time.strftime('%Y-%m-%d %H:%M:%S')} - {msg}\n")
                                    root.after(0, dismiss_reminders)
                                else:
                                    append_debug_log(debug_log, f"{time.strftime('%Y-%m-%d %H:%M:%S')} - Ignored in reminders mode: '{line}'\n")

                            elif calibration_mode:
                                if calibration_state and calibration_state.get("phase") == "review" and line == '+1':
                                    msg = "Serial: Calibration reread"
                                    print(msg)
                                    append_debug_log(debug_log, f"{time.strftime('%Y-%m-%d %H:%M:%S')} - {msg}\n")
                                    root.after(0, calibration_reread_now)
                                elif line in ('+1', '-1', '+10', '-10'):
                                    msg = f"Serial: Calibration command {line}"
                                    print(msg)
                                    append_debug_log(debug_log, f"{time.strftime('%Y-%m-%d %H:%M:%S')} - {msg}\n")
                                    root.after(0, calibration_adjust_value, int(line))
                                elif line == 'OV':
                                    msg = "Serial: Calibration confirm"
                                    print(msg)
                                    append_debug_log(debug_log, f"{time.strftime('%Y-%m-%d %H:%M:%S')} - {msg}\n")
                                    root.after(0, calibration_confirm)
                                elif line == 'PS':
                                    msg = "Serial: Calibration cancel/back"
                                    print(msg)
                                    append_debug_log(debug_log, f"{time.strftime('%Y-%m-%d %H:%M:%S')} - {msg}\n")
                                    root.after(0, calibration_cancel)
                                else:
                                    append_debug_log(debug_log, f"{time.strftime('%Y-%m-%d %H:%M:%S')} - Ignored in calibration mode: '{line}'\n")

                            # Handle log viewer navigation if in log viewer mode
                            elif log_viewer_mode:
                                if line == '+1':
                                    msg = "Serial: Log viewer scroll down"
                                    print(msg)
                                    append_debug_log(debug_log, f"{time.strftime('%Y-%m-%d %H:%M:%S')} - {msg}\n")
                                    root.after(0, log_viewer_scroll_down)
                                elif line == '-1':
                                    msg = "Serial: Log viewer scroll up"
                                    print(msg)
                                    append_debug_log(debug_log, f"{time.strftime('%Y-%m-%d %H:%M:%S')} - {msg}\n")
                                    root.after(0, log_viewer_scroll_up)
                                elif line == 'OV':
                                    msg = "Serial: Log viewer exit"
                                    print(msg)
                                    append_debug_log(debug_log, f"{time.strftime('%Y-%m-%d %H:%M:%S')} - {msg}\n")
                                    root.after(0, close_log_viewer)
                                else:
                                    append_debug_log(debug_log, f"{time.strftime('%Y-%m-%d %H:%M:%S')} - Ignored in log viewer mode: '{line}'\n")

                            # Handle fill history navigation if in fill history mode
                            elif fill_history_mode:
                                if line == '+1':
                                    msg = "Serial: Fill history scroll down"
                                    print(msg)
                                    append_debug_log(debug_log, f"{time.strftime('%Y-%m-%d %H:%M:%S')} - {msg}\n")
                                    root.after(0, fill_history_scroll_down)
                                elif line == '-1':
                                    msg = "Serial: Fill history scroll up"
                                    print(msg)
                                    append_debug_log(debug_log, f"{time.strftime('%Y-%m-%d %H:%M:%S')} - {msg}\n")
                                    root.after(0, fill_history_scroll_up)
                                elif line == 'OV':
                                    msg = "Serial: Fill history exit"
                                    print(msg)
                                    append_debug_log(debug_log, f"{time.strftime('%Y-%m-%d %H:%M:%S')} - {msg}\n")
                                    root.after(0, close_fill_history)
                                else:
                                    append_debug_log(debug_log, f"{time.strftime('%Y-%m-%d %H:%M:%S')} - Ignored in fill history mode: '{line}'\n")

                            # Handle self-test mode
                            elif self_test_mode:
                                if line == 'OV':
                                    msg = "Serial: Self-test exit"
                                    print(msg)
                                    append_debug_log(debug_log, f"{time.strftime('%Y-%m-%d %H:%M:%S')} - {msg}\n")
                                    root.after(0, close_self_test)
                                else:
                                    append_debug_log(debug_log, f"{time.strftime('%Y-%m-%d %H:%M:%S')} - Ignored in self-test mode: '{line}'\n")

                            # Handle full-test mode
                            elif full_test_mode:
                                if line == 'OV':
                                    # Check if OV test has been done already
                                    if full_test_window and hasattr(full_test_window, 'test_status'):
                                        if not full_test_window.test_status['OV']:
                                            # First press: mark OV as tested
                                            msg = "Serial: Full-test OV command detected (marked as tested)"
                                            print(msg)
                                            append_debug_log(debug_log, f"{time.strftime('%Y-%m-%d %H:%M:%S')} - {msg}\n")
                                            root.after(0, lambda: full_test_window.mark_tested('OV'))
                                        else:
                                            # Second press: exit full test
                                            msg = "Serial: Full-test exit (OV pressed second time)"
                                            print(msg)
                                            append_debug_log(debug_log, f"{time.strftime('%Y-%m-%d %H:%M:%S')} - {msg}\n")
                                            root.after(0, close_full_test)
                                    else:
                                        # Fallback: just exit
                                        msg = "Serial: Full-test exit (OV pressed)"
                                        print(msg)
                                        append_debug_log(debug_log, f"{time.strftime('%Y-%m-%d %H:%M:%S')} - {msg}\n")
                                        root.after(0, close_full_test)
                                elif line in ['-1', '+1', '-10', '+10', 'PS']:
                                    msg = f"Serial: Full-test {line} command detected"
                                    print(msg)
                                    append_debug_log(debug_log, f"{time.strftime('%Y-%m-%d %H:%M:%S')} - {msg}\n")
                                    # Mark the corresponding test as passed
                                    if full_test_window and hasattr(full_test_window, 'mark_tested'):
                                        if line == '-1':
                                            root.after(0, lambda: full_test_window.mark_tested('minus_1'))
                                        elif line == '+1':
                                            root.after(0, lambda: full_test_window.mark_tested('plus_1'))
                                        elif line == '-10':
                                            root.after(0, lambda: full_test_window.mark_tested('minus_10'))
                                        elif line == '+10':
                                            root.after(0, lambda: full_test_window.mark_tested('plus_10'))
                                        elif line == 'PS':
                                            root.after(0, lambda: full_test_window.mark_tested('PS'))
                                else:
                                    append_debug_log(debug_log, f"{time.strftime('%Y-%m-%d %H:%M:%S')} - Ignored in full-test mode: '{line}'\n")

                            # Handle update mode
                            elif update_mode:
                                if line == 'OV':
                                    msg = "Serial: Update exit"
                                    print(msg)
                                    append_debug_log(debug_log, f"{time.strftime('%Y-%m-%d %H:%M:%S')} - {msg}\n")
                                    root.after(0, close_update)
                                else:
                                    append_debug_log(debug_log, f"{time.strftime('%Y-%m-%d %H:%M:%S')} - Ignored in update mode: '{line}'\n")

                            # Handle menu navigation if in menu mode
                            elif menu_mode:
                                if line == '+1':
                                    msg = "Serial: Menu navigate down"
                                    print(msg)
                                    append_debug_log(debug_log, f"{time.strftime('%Y-%m-%d %H:%M:%S')} - {msg}\n")
                                    menu_navigate_down()
                                elif line == '-1':
                                    msg = "Serial: Menu navigate up"
                                    print(msg)
                                    append_debug_log(debug_log, f"{time.strftime('%Y-%m-%d %H:%M:%S')} - {msg}\n")
                                    menu_navigate_up()
                                elif line == 'OV':
                                    msg = "Serial: Menu select"
                                    print(msg)
                                    append_debug_log(debug_log, f"{time.strftime('%Y-%m-%d %H:%M:%S')} - {msg}\n")
                                    arm_menu_ov_guard()
                                    root.after(0, menu_select)
                                else:
                                    append_debug_log(debug_log, f"{time.strftime('%Y-%m-%d %H:%M:%S')} - Ignored in menu mode: '{line}'\n")

                            # Normal dashboard mode
                            else:
                                # Handle adjustment commands
                                if line in ['+1', '-1', '+10', '-10']:
                                    try:
                                        adjustment = int(line)
                                        if adjust_batch_mix_gallons(adjustment, "Serial"):
                                            continue
                                        requested_gallons += adjustment
                                        # Don't allow requested gallons to go below zero
                                        if requested_gallons < 0:
                                            requested_gallons = 0
                                        colors_are_green = False  # Reset colors when requested gallons changes
                                        # Update the current mode's preset
                                        if current_mode == 'fill':
                                            fill_requested_gallons = requested_gallons
                                        else:
                                            mix_requested_gallons = requested_gallons
                                        save_mode_presets()
                                        heartbeat_age = time.time() - last_heartbeat_time if last_heartbeat_time else -1
                                        msg = (
                                            f"Serial: Adjusted by {adjustment}, requested gallons now {requested_gallons} "
                                            f"(heartbeat_age={heartbeat_age:.1f}s, mode={current_mode})"
                                        )
                                        print(msg)
                                        log_serial_debug(msg)
                                        root.after(
                                            0,
                                            lambda value=requested_gallons, is_green=colors_are_green: (
                                                draw_requested_number(f"{value:.0f}", "green" if is_green else "red")
                                                if batch_mix_layout_active
                                                else (draw_requested_number(f"{value:.0f}", "green" if is_green else "red"), update_batch_mix_overlay())
                                            ),
                                        )
                                    except ValueError as ve:
                                        log_serial_debug(f"ValueError parsing adjustment: {ve}")

                                # Handle special commands
                                elif line == 'PS':
                                    msg = "Serial: Pump Stop command received - activating relay"
                                    print(msg)
                                    append_debug_log(debug_log, f"{time.strftime('%Y-%m-%d %H:%M:%S')} - {msg}\n")
                                    start_pump_stop_thread(config.PUMP_STOP_DURATION)

                                elif line == 'OV':
                                    global override_enabled_time
                                    # Check if requested gallons is 0 to trigger menu or leave batch mix screen.
                                    if requested_gallons == 0:
                                        if current_mode == 'mix' and batch_mix_data is not None:
                                            msg = "Serial: Batch mix screen exit triggered (gallons=0, OV pressed)"
                                            print(msg)
                                            append_debug_log(debug_log, f"{time.strftime('%Y-%m-%d %H:%M:%S')} - {msg}\n")
                                            root.after(0, lambda: clear_batch_mix_screen("serial OV at zero gallons"))
                                        else:
                                            msg = "Serial: Menu access triggered (gallons=0, OV pressed)"
                                            print(msg)
                                            append_debug_log(debug_log, f"{time.strftime('%Y-%m-%d %H:%M:%S')} - {msg}\n")
                                            # Show menu in main thread
                                            arm_menu_ov_guard()
                                            root.after(0, show_menu)
                                    else:
                                        now = time.monotonic()
                                        if now - last_serial_ov_toggle_time < SERIAL_OV_TOGGLE_DEBOUNCE_SECONDS:
                                            msg = "Serial: Ignored OV debounce during override toggle"
                                            print(msg)
                                            log_serial_debug(msg)
                                            append_debug_log(debug_log, f"{time.strftime('%Y-%m-%d %H:%M:%S')} - {msg}\n")
                                            continue
                                        last_serial_ov_toggle_time = now
                                        override_mode = not override_mode
                                        if override_mode:
                                            override_enabled_time = time.time()  # Record when enabled
                                        msg = f"Serial: Override mode {'ENABLED' if override_mode else 'DISABLED'}"
                                        print(msg)
                                        append_debug_log(debug_log, f"{time.strftime('%Y-%m-%d %H:%M:%S')} - {msg}\n")


                                elif line == 'TU':
                                    msg = "Serial: Thumbs Up command received"
                                    print(msg)
                                    append_debug_log(debug_log, f"{time.strftime('%Y-%m-%d %H:%M:%S')} - {msg}\n")
                                    root.after(0, lambda: handle_thumbs_up_press("serial TU"))

                                elif line in ('RST', 'RESET'):
                                    msg = "Serial: Flow reset command received"
                                    print(msg)
                                    append_debug_log(debug_log, f"{time.strftime('%Y-%m-%d %H:%M:%S')} - {msg}\n")
                                    root.after(0, lambda: force_flow_reset("serial reset"))

                                elif line == 'MIX':
                                    msg = "Serial: Mix mode command received"
                                    print(msg)
                                    append_debug_log(debug_log, f"{time.strftime('%Y-%m-%d %H:%M:%S')} - {msg}\n")
                                    root.after(0, lambda: switch_mode('mix'))

                                elif line == 'FILL':
                                    msg = "Serial: Fill mode command received"
                                    print(msg)
                                    append_debug_log(debug_log, f"{time.strftime('%Y-%m-%d %H:%M:%S')} - {msg}\n")
                                    root.after(0, lambda: switch_mode('fill'))

                                else:
                                    # Unknown command
                                    append_debug_log(debug_log, f"{time.strftime('%Y-%m-%d %H:%M:%S')} - Unknown command: '{line}'\n")
                else:
                    time.sleep(0.1)
            except Exception as e:
                msg = f"Serial read error: {e}"
                print(msg)
                log_serial_debug(msg)
                time.sleep(0.1)

    except Exception as e:
        serial_connected = False
        msg = f"Serial listener error: {e}"
        print(msg)
        log_serial_debug(msg)

def initialize_gpio():
    """Initialize GPIO for relay control"""
    if not GPIO_AVAILABLE:
        print("GPIO not available, skipping GPIO initialization")
        return True

    try:
        # Disable warnings about channels already in use
        GPIO.setwarnings(False)
        # Configure GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(config.PUMP_STOP_RELAY_PIN, GPIO.OUT)
        GPIO.setup(config.FLOW_RESET_PIN, GPIO.OUT)
        GPIO.output(config.FLOW_RESET_PIN, GPIO.LOW)
        GPIO.output(config.PUMP_STOP_RELAY_PIN, GPIO.LOW)
        print(f"GPIO initialized: Relay on pin {config.PUMP_STOP_RELAY_PIN}")
        return True
    except Exception as e:
        print(f"Failed to initialize GPIO: {e}")
        return False

# Flow meter reset
flow_reset_scheduled = False
flow_reset_cycle_id = None
flow_cycle_counter = 0

def reset_is_blocked_by_flow():
    """Block reset for forward flow or invalid data; permit reverse flow."""
    if not math.isfinite(last_flow_rate):
        return True
    return last_flow_rate >= config.FLOW_METER_ZERO_THRESHOLD


def clear_auto_shutoff_state(reason=""):
    """Clear per-cycle auto-shutoff state after a reset or new flow cycle."""
    global last_alert_triggered, auto_shutoff_latched
    global last_trigger_flow_gpm, last_trigger_threshold, last_trigger_actual
    global last_trigger_predicted_actual, last_trigger_loop_dt_ms
    global last_pump_stop_relay_activated_at

    last_alert_triggered = False
    auto_shutoff_latched = False
    last_trigger_flow_gpm = 0.0
    last_trigger_threshold = 0.0
    last_trigger_actual = 0.0
    last_trigger_predicted_actual = 0.0
    last_trigger_loop_dt_ms = 0.0
    last_pump_stop_relay_activated_at = 0.0
    recent_flow_rates_l_per_s.clear()
    if reason:
        print(f"Auto-shutoff state cleared: {reason}")


def _mark_expected_totalizer_reset():
    """Expect one observable nonzero-counter transition after a Box reset pulse."""
    global expected_totalizer_reset_until, expected_totalizer_reset_from_gallons

    counter_gallons = last_totalizer_liters * config.LITERS_TO_GALLONS
    if not math.isfinite(counter_gallons) or abs(counter_gallons) < 0.25:
        expected_totalizer_reset_until = 0.0
        expected_totalizer_reset_from_gallons = 0.0
        return False
    expected_totalizer_reset_from_gallons = counter_gallons
    expected_totalizer_reset_until = time.time() + 1.0
    return True


def _activate_physical_reset_safety(previous_gallons, flow_rate_l_per_s):
    """Stop the pump and retain the first unexpected in-flow reset for acknowledgement."""
    global physical_reset_safety_active
    global physical_reset_gallons_at_event, physical_reset_flow_gpm_at_event

    flow_gpm = flow_rate_l_per_s * config.LITERS_PER_SEC_TO_GPM
    if physical_reset_safety_active:
        log_flow_control(
            "physical_reset_during_flow_repeated"
            f" | gallons_before={previous_gallons:.3f}"
            f" | flow_gpm={flow_gpm:.2f}"
        )
        return False

    physical_reset_safety_active = True
    physical_reset_gallons_at_event = max(0.0, previous_gallons)
    physical_reset_flow_gpm_at_event = flow_gpm
    msg = (
        "Physical meter reset detected during forward flow"
        f" ({physical_reset_gallons_at_event:.1f} gal at reset, {flow_gpm:.1f} GPM)"
    )
    print(msg)
    log_serial_debug(msg)
    log_flow_control(
        "physical_reset_during_flow"
        f" | gallons_before={physical_reset_gallons_at_event:.3f}"
        f" | flow_gpm={flow_gpm:.2f}"
        " | pump_stop"
    )
    start_pump_stop_thread(config.PUMP_STOP_DURATION)
    try:
        root.after(0, lambda: _show_physical_reset_safety_window())
    except Exception:
        pass
    return True


def clear_physical_reset_safety(source="pump stop acknowledgment"):
    """Acknowledge an in-flow physical-reset incident without resetting the meter again."""
    global physical_reset_safety_active

    if not physical_reset_safety_active:
        return False
    physical_reset_safety_active = False
    msg = f"Physical reset safety cleared by {source}"
    print(msg)
    log_serial_debug(msg)
    log_flow_control(
        "physical_reset_safety_cleared"
        f" | source={source}"
        f" | gallons_before={physical_reset_gallons_at_event:.3f}"
    )
    try:
        root.after(0, lambda: _hide_physical_reset_safety_window())
    except Exception:
        pass
    return True


def detect_totalizer_reset(totalizer_liters, flow_rate_l_per_s=None):
    """Detect a totalizer reset and stop on an unexpected reset during forward flow."""
    global previous_totalizer_liters, flow_cycle_counter
    global expected_totalizer_reset_until, expected_totalizer_reset_from_gallons

    previous_gallons = previous_totalizer_liters * config.LITERS_TO_GALLONS
    current_gallons = totalizer_liters * config.LITERS_TO_GALLONS
    reset_drop_gallons = previous_gallons - current_gallons
    reset_detected = (
        (abs(previous_gallons) > 0.25 and abs(current_gallons) < 0.05)
        or reset_drop_gallons >= 0.25
    )
    now = time.time()
    expected_counter_matches = bool(
        abs(previous_gallons - expected_totalizer_reset_from_gallons) <= 0.25
    )
    expected_reset = bool(
        reset_detected
        and expected_totalizer_reset_until > 0.0
        and now <= expected_totalizer_reset_until
        and expected_counter_matches
    )
    if reset_detected or now > expected_totalizer_reset_until:
        expected_totalizer_reset_until = 0.0
        expected_totalizer_reset_from_gallons = 0.0
    previous_totalizer_liters = totalizer_liters

    if reset_detected:
        current_flow = last_flow_rate if flow_rate_l_per_s is None else flow_rate_l_per_s
        positive_flow = bool(
            math.isfinite(current_flow)
            and current_flow >= config.FLOW_METER_ZERO_THRESHOLD
        )
        if positive_flow and not expected_reset:
            _activate_physical_reset_safety(previous_gallons, current_flow)
        elif expected_reset:
            log_flow_control(
                "expected_box_totalizer_reset_observed"
                f" | gallons_before={previous_gallons:.3f}"
                f" | gallons_after={current_gallons:.3f}"
                f" | flow_lps={current_flow:.4f}"
            )
        flow_cycle_counter += 1
        clear_auto_shutoff_state("totalizer reset to zero")
        print(
            f"New fill cycle assumed from totalizer reset "
            f"({previous_gallons:.3f} -> {current_gallons:.3f} gal)"
        )


def _pulse_flow_reset_gpio(reason="flow reset"):
    global flow_reset_scheduled, flow_reset_cycle_id
    global expected_totalizer_reset_until, expected_totalizer_reset_from_gallons
    if reset_is_blocked_by_flow():
        msg = f"Flow reset blocked: forward flow active ({last_flow_rate:.3f} L/s)"
        print(msg)
        log_serial_debug(msg)
        with open("/home/pi/reset_debug.log", "a") as dbg:
            dbg.write(msg + "\n")
        flow_reset_scheduled = False
        flow_reset_cycle_id = None
        return False
    if SIM_MODE:
        _mark_expected_totalizer_reset()
        _sim_flow_reset(reason)
        return True
    if not GPIO_AVAILABLE:
        flow_reset_scheduled = False
        flow_reset_cycle_id = None
        return False
    try:
        with open("/home/pi/reset_debug.log", "a") as dbg:
            dbg.write("pulsing gpio0\n")
        _mark_expected_totalizer_reset()
        GPIO.output(config.FLOW_RESET_PIN, GPIO.HIGH)
        time.sleep(config.FLOW_RESET_DURATION)
        GPIO.output(config.FLOW_RESET_PIN, GPIO.LOW)
        with open("/home/pi/reset_debug.log", "a") as dbg:
            dbg.write("done\n")
    except Exception as e:
        expected_totalizer_reset_until = 0.0
        expected_totalizer_reset_from_gallons = 0.0
        with open("/home/pi/reset_debug.log", "a") as dbg:
            dbg.write(str(e) + "\n")
        flow_reset_scheduled = False
        flow_reset_cycle_id = None
        return False
    flow_reset_scheduled = False
    flow_reset_cycle_id = None
    return True


def _sim_flow_reset(reason):
    global flow_reset_scheduled, flow_reset_cycle_id, last_totalizer_liters
    iolhat.reset_totalizer()
    detect_totalizer_reset(0.0)
    last_totalizer_liters = 0.0
    draw_actual_number("0.0", target_display_color(0.0))
    with open("/home/pi/reset_debug.log", "a") as dbg:
        dbg.write(f"sim totalizer reset: {reason}\n")
    flow_reset_scheduled = False
    flow_reset_cycle_id = None


def force_flow_reset(reason="forced"):
    global flow_reset_scheduled, flow_reset_cycle_id
    msg = f"Flow reset requested: {reason} ({last_flow_rate:.3f} L/s)"
    print(msg)
    log_serial_debug(msg)
    with open("/home/pi/reset_debug.log", "a") as dbg:
        dbg.write(msg + "\n")
    reset_completed = _pulse_flow_reset_gpio(reason)
    if reset_completed:
        clear_auto_shutoff_state(reason)
    return reset_completed


def handle_box_pump_stop_button(reason="box pump stop"):
    """Reassert pump stop and immediately acknowledge a physical-reset incident."""
    start_pump_stop_thread(config.PUMP_STOP_DURATION)
    return clear_physical_reset_safety(reason)


def pulse_flow_reset():
    global flow_reset_scheduled, flow_reset_cycle_id
    with open("/home/pi/reset_debug.log", "a") as dbg:
        dbg.write("pulse called\n")
    if flow_reset_cycle_id != flow_cycle_counter:
        msg = "Flow reset cancelled: new flow started after reset was requested"
        print(msg)
        log_serial_debug(msg)
        with open("/home/pi/reset_debug.log", "a") as dbg:
            dbg.write(msg + "\n")
        flow_reset_scheduled = False
        flow_reset_cycle_id = None
        return
    _pulse_flow_reset_gpio("scheduled pulse")

def schedule_flow_reset():
    global flow_reset_scheduled, flow_reset_cycle_id
    if flow_reset_scheduled:
        return
    if reset_is_blocked_by_flow():
        msg = f"Flow reset not scheduled: forward flow active ({last_flow_rate:.3f} L/s)"
        print(msg)
        log_serial_debug(msg)
        with open("/home/pi/reset_debug.log", "a") as dbg:
            dbg.write(msg + "\n")
        return
    flow_reset_scheduled = True
    flow_reset_cycle_id = flow_cycle_counter
    root.after(int(config.FLOW_RESET_DELAY * 1000), pulse_flow_reset)


def initialize_iol():
    """Initialize IO-Link port"""
    try:
        with iol_io_lock:
            # Power on the port
            iolhat.power(config.IOL_PORT, 1)
            # Set LED to green
            iolhat.led(config.IOL_PORT, iolhat.LED_GREEN)
        time.sleep(0.5)
        print(f"IO-Link Port {config.IOL_PORT+1} initialized successfully")
        return True
    except Exception as e:
        print(f"Failed to initialize IO-Link Port {config.IOL_PORT+1}: {e}")
        return False

# Tkinter GUI setup
root = tk.Tk()
root.title("Tank Dashboard")
root.configure(bg="black")
if SIM_MODE:
    screen_w = root.winfo_screenwidth()
    screen_h = root.winfo_screenheight()
    SIM_RENDER_SCALE = min(
        max(0.1, (screen_w - 24) / SIM_WINDOW_WIDTH),
        max(0.1, (screen_h - 96) / SIM_WINDOW_HEIGHT),
        1.0,
    )
    sim_display_width = max(1, int(SIM_WINDOW_WIDTH * SIM_RENDER_SCALE))
    sim_display_height = max(1, int(SIM_WINDOW_HEIGHT * SIM_RENDER_SCALE))
    root.geometry(f"{sim_display_width}x{sim_display_height}")
    root.resizable(True, True)
else:
    root.attributes("-fullscreen", True)

# Get screen dimensions
screen_width = root.winfo_screenwidth()
screen_height = root.winfo_screenheight()

# Create ONE full-screen canvas for everything
canvas = tk.Canvas(
    root,
    bg="black",
    highlightthickness=0,
    width=sim_display_width if SIM_MODE else 0,
    height=sim_display_height if SIM_MODE else 0,
)
canvas.pack(fill='both', expand=True)


def _sim_scale_value(value):
    if SIM_MODE and isinstance(value, (int, float)):
        return value * SIM_RENDER_SCALE
    return value


def _sim_scale_args(args):
    return tuple(_sim_scale_value(arg) for arg in args)


def _sim_scale_font(font):
    if not SIM_MODE or not isinstance(font, tuple) or len(font) < 2:
        return font
    size = font[1]
    if not isinstance(size, int):
        return font
    scaled_size = max(1, int(round(abs(size) * SIM_RENDER_SCALE)))
    return (font[0], -scaled_size, *font[2:])


if SIM_MODE:
    _canvas_create_text = canvas.create_text
    _canvas_create_rectangle = canvas.create_rectangle
    _canvas_create_line = canvas.create_line

    def _sim_create_text(*args, **kwargs):
        if "font" in kwargs:
            kwargs["font"] = _sim_scale_font(kwargs["font"])
        return _canvas_create_text(*_sim_scale_args(args), **kwargs)

    def _sim_create_rectangle(*args, **kwargs):
        return _canvas_create_rectangle(*_sim_scale_args(args), **kwargs)

    def _sim_create_line(*args, **kwargs):
        return _canvas_create_line(*_sim_scale_args(args), **kwargs)

    canvas.create_text = _sim_create_text
    canvas.create_rectangle = _sim_create_rectangle
    canvas.create_line = _sim_create_line

def _sync_canvas_geometry():
    if not SIM_MODE:
        canvas.update()


def _canvas_width():
    width = canvas.winfo_width()
    return SIM_WINDOW_WIDTH if SIM_MODE else width


def _canvas_height():
    height = canvas.winfo_height()
    return SIM_WINDOW_HEIGHT if SIM_MODE else height


def physical_reset_safety_message():
    """Operator instructions for an unexpected meter reset during forward flow."""
    return (
        "PUMP STOPPED - METER RESET DURING FLOW\n"
        f"METER HAD {physical_reset_gallons_at_event:.1f} GAL WHEN RESET WAS PRESSED\n"
        "DO NOT RESET THE METER WHILE FLOWING\n"
        "PRESS PUMP STOP TO ACKNOWLEDGE"
    )


def draw_physical_reset_safety_banner():
    """Draw a persistent compact warning without hiding requested or actual gallons."""
    canvas.delete("physical_reset_safety")
    if not physical_reset_safety_active:
        return

    width = _canvas_width()
    height = _canvas_height()
    banner_top = max(0, height - 175)
    banner_bottom = max(banner_top + 1, height - 40)
    canvas.create_rectangle(
        20,
        banner_top,
        width - 20,
        banner_bottom,
        fill="black",
        outline="yellow",
        width=5,
        tags="physical_reset_safety",
    )
    canvas.create_text(
        width // 2,
        (banner_top + banner_bottom) // 2,
        text=physical_reset_safety_message(),
        font=("Helvetica", max(20, int(height * 0.025)), "bold"),
        fill="yellow",
        justify="center",
        tags="physical_reset_safety",
    )
    canvas.tag_raise("physical_reset_safety")


def _show_physical_reset_safety_window():
    """Keep the reset warning visible above fullscreen menus and workflows."""
    global physical_reset_safety_window, physical_reset_safety_label

    if not physical_reset_safety_active:
        _hide_physical_reset_safety_window()
        return
    try:
        window_exists = bool(
            physical_reset_safety_window is not None
            and physical_reset_safety_window.winfo_exists()
        )
        if not window_exists:
            physical_reset_safety_window = tk.Toplevel(root)
            physical_reset_safety_window.overrideredirect(True)
            physical_reset_safety_window.configure(bg="black")
            physical_reset_safety_label = tk.Label(
                physical_reset_safety_window,
                bg="black",
                fg="yellow",
                justify="center",
                bd=5,
                relief="solid",
                highlightbackground="yellow",
                highlightcolor="yellow",
                highlightthickness=3,
            )
            physical_reset_safety_label.pack(fill="both", expand=True)

        root.update_idletasks()
        root_width = max(1, root.winfo_width())
        root_height = max(1, root.winfo_height())
        banner_width = max(400, root_width - 40)
        banner_height = max(110, min(150, int(root_height * 0.14)))
        banner_x = root.winfo_rootx() + max(0, (root_width - banner_width) // 2)
        banner_y = root.winfo_rooty() + max(0, root_height - banner_height - 35)
        font_size = max(16, min(28, int(root_height * 0.025)))
        physical_reset_safety_label.configure(
            text=physical_reset_safety_message(),
            font=("Helvetica", font_size, "bold"),
            wraplength=max(200, banner_width - 30),
        )
        physical_reset_safety_window.geometry(
            f"{banner_width}x{banner_height}+{banner_x}+{banner_y}"
        )
        physical_reset_safety_window.deiconify()
        physical_reset_safety_window.lift()
        try:
            physical_reset_safety_window.attributes("-topmost", True)
        except Exception:
            pass
    except Exception as exc:
        log_flow_control(f"physical_reset_safety_window_error | error={exc}")


def _hide_physical_reset_safety_window():
    """Hide the cross-window reset warning after local acknowledgement."""
    try:
        if (
            physical_reset_safety_window is not None
            and physical_reset_safety_window.winfo_exists()
        ):
            physical_reset_safety_window.withdraw()
    except Exception:
        pass


def update_negative_totalizer_fault_flash():
    """Flash a top-screen reset-required warning for negative totalizer drift."""
    global negative_totalizer_alarm_visible

    flash_hz = 4.0
    interval_ms = max(1, int(1000 / flash_hz))

    if negative_totalizer_fault_active or negative_flow_fault_active or positive_drift_fault_active:
        phase = int(time.time() * flash_hz) % 2
        bg = "red" if phase == 0 else "black"
        fg = "white" if phase == 0 else "yellow"
        canvas.delete("negative_totalizer_alarm")
        width = _canvas_width()
        height = _canvas_height()
        band_height = max(260, int(height * 0.44))
        signed_flow_gpm = last_flow_rate * config.LITERS_PER_SEC_TO_GPM
        if negative_totalizer_fault_active or negative_flow_fault_active:
            warning_text = (
                "NEGATIVE FLOW METER\n"
                f"TOTALIZER: {last_negative_totalizer_gallons:.1f} GAL\n"
                f"FLOW: {signed_flow_gpm:.1f} GPM\n"
                "GALLON RESET REQUIRED\n"
                "PUMP WILL NOT START UNTIL RESET"
            )
        else:
            warning_text = (
                "FLOW METER DRIFT\n"
                f"DRIFT: +{positive_drift_gallons:.1f} GAL\n"
                f"FLOW: {positive_drift_flow_gpm:.1f} GPM\n"
                "GALLON RESET REQUIRED\n"
                + ("OVERRIDE ACTIVE" if override_mode else "PUMP STOPPED - OVERRIDE ALLOWED")
            )
        canvas.create_rectangle(
            0,
            0,
            width,
            band_height,
            fill=bg,
            outline="",
            tags="negative_totalizer_alarm",
        )
        canvas.create_text(
            width // 2,
            band_height // 2,
            text=warning_text,
            font=("Helvetica", max(38, int(band_height * 0.14)), "bold"),
            fill=fg,
            justify="center",
            tags="negative_totalizer_alarm",
        )
        canvas.tag_raise("negative_totalizer_alarm")
        negative_totalizer_alarm_visible = True
    elif negative_totalizer_alarm_visible:
        canvas.delete("negative_totalizer_alarm")
        negative_totalizer_alarm_visible = False

    root.after(interval_ms, update_negative_totalizer_fault_flash)


def update_relay_slowdown_alarm_flash():
    """Flash the whole display while auto-stop relay has not slowed flow."""
    global relay_slowdown_alarm_visible

    flash_hz = max(1.0, float(getattr(config, "RELAY_SLOWDOWN_ALARM_FLASH_HZ", 30)))
    interval_ms = max(1, int(1000 / flash_hz))

    if relay_slowdown_alarm_active:
        phase = int(time.time() * flash_hz) % 2
        color = (
            config.RELAY_SLOWDOWN_ALARM_COLOR_A
            if phase == 0
            else config.RELAY_SLOWDOWN_ALARM_COLOR_B
        )
        canvas.delete("relay_slowdown_alarm")
        width = _canvas_width()
        height = _canvas_height()
        canvas.create_rectangle(
            0,
            0,
            width,
            height,
            fill=color,
            outline="",
            tags="relay_slowdown_alarm",
        )
        # Keep a steady red caution symbol centered over the flashing background.
        # The background still flashes black/white to show the pump-stop failure,
        # but the warning glyph itself does not alternate color.
        canvas.create_text(
            width // 2,
            height // 2,
            text="⚠",
            font=("Helvetica", max(160, int(min(width, height) * 0.55)), "bold"),
            fill="red",
            tags="relay_slowdown_alarm",
        )
        canvas.tag_raise("relay_slowdown_alarm")
        relay_slowdown_alarm_visible = True
    elif relay_slowdown_alarm_visible:
        canvas.delete("relay_slowdown_alarm")
        relay_slowdown_alarm_visible = False

    root.after(interval_ms, update_relay_slowdown_alarm_flash)


# Draw full-screen barber pole stripes
def draw_fullscreen_stripes():
    """Draw barber pole stripes across entire screen"""
    # Get actual canvas size after it's been packed
    _sync_canvas_geometry()
    width = _canvas_width()
    height = _canvas_height()

    stripe_height = 30
    dark_yellow = "#CC9900"

    for i, stripe_y in enumerate(range(0, height, stripe_height)):
        stripe_color = "red" if i % 2 == 0 else dark_yellow
        canvas.create_rectangle(0, stripe_y, width, stripe_y + stripe_height,
                               fill=stripe_color, outline="", tags="stripes")

# Draw stripes after window is created
if not SIM_MODE:
    root.update()
# draw_fullscreen_stripes()  # Disabled - using solid black background


def _source_observation_epoch(value):
    """Normalize a box-supplied sensor timestamp without inventing one."""
    try:
        observed_at = float(value)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(observed_at) or observed_at <= 0:
        return None
    return observed_at


def _parse_mopeka_sensor_command(line):
    """Parse one additive tank command using the exact wire prefix length."""
    prefix = "MOPEKA_SENSOR:"
    if not line.startswith(prefix):
        return None
    parts = line[len(prefix):].split("|")
    if len(parts) < 3:
        raise ValueError("Mopeka sensor command is missing required fields")

    index = int(parts[0])
    if index not in (1, 2):
        raise ValueError(f"Invalid Mopeka sensor index: {index}")
    gallons = float(parts[1])
    if not math.isfinite(gallons):
        raise ValueError("Invalid Mopeka gallons")
    quality = int(parts[2])
    observed_at = parts[3] if len(parts) > 3 and parts[3] else None
    identity_token = parts[6] if len(parts) > 6 and parts[6] else None

    optional_values = []
    for position in (4, 5):
        value = None
        if len(parts) > position and parts[position]:
            value = float(parts[position])
            if not math.isfinite(value):
                raise ValueError("Invalid Mopeka level")
        optional_values.append(value)

    return (
        index,
        gallons,
        quality,
        observed_at,
        optional_values[0],
        optional_values[1],
        identity_token,
    )


def _accept_trailer_sensor_identity(identity_token):
    """Bind an observed reading to its scanner process identity."""
    global trailer_sensor_identity
    if identity_token is None:
        # Timestamp-less/identity-less senders remain readable locally, but do
        # not gain a cache owner that RotorLink could associate cross-device.
        return True
    normalized = normalize_history_identity_token(identity_token)
    if normalized is None:
        return False
    if trailer_sensor_identity != normalized:
        # The dashboard cache is aggregate, so a single new-owner reading must
        # not relabel untouched fields from the prior trailer. Clear every
        # sensor group first, then let the caller apply only this observation.
        _reset_trailer_sensor_telemetry()
    trailer_sensor_identity = normalized
    return True


def _reset_trailer_sensor_telemetry():
    """Clear sensor caches only for a confirmed trailer identity change."""
    global mopeka1_gallons, mopeka2_gallons, mopeka1_quality, mopeka2_quality
    global mopeka1_level_mm, mopeka2_level_mm, mopeka1_level_in, mopeka2_level_in
    global mopeka1_has_reading, mopeka2_has_reading
    global mopeka1_observed_at, mopeka2_observed_at, mopeka_connected
    global bms_soc, bms_voltage, bms_has_reading, bms_observed_at

    mopeka1_gallons = 0.0
    mopeka2_gallons = 0.0
    mopeka1_quality = 0
    mopeka2_quality = 0
    mopeka1_level_mm = 0.0
    mopeka2_level_mm = 0.0
    mopeka1_level_in = 0.0
    mopeka2_level_in = 0.0
    mopeka1_has_reading = False
    mopeka2_has_reading = False
    mopeka1_observed_at = None
    mopeka2_observed_at = None
    mopeka_connected = False

    bms_soc = None
    bms_voltage = None
    bms_has_reading = False
    bms_observed_at = None


def _mark_trailer_sensor_identity_changed():
    global trailer_sensor_identity_generation
    global trailer_sensor_identity
    durable_identity = _read_trailer_sensor_identity_token()
    if trailer_sensor_identity != durable_identity:
        _reset_trailer_sensor_telemetry()
    trailer_sensor_identity = durable_identity
    trailer_sensor_identity_generation += 1


def _apply_mopeka(
    m1g,
    m2g,
    m1q,
    m2q,
    m1_observed_at=None,
    m2_observed_at=None,
    identity_token=None,
):
    """Apply mopeka values and update display (called from main thread via root.after)"""
    global mopeka1_gallons, mopeka2_gallons, mopeka1_quality, mopeka2_quality
    global mopeka_connected, mopeka_enabled
    global mopeka1_has_reading, mopeka2_has_reading
    global mopeka1_observed_at, mopeka2_observed_at
    if not _accept_trailer_sensor_identity(identity_token):
        return
    mopeka1_gallons = m1g
    mopeka2_gallons = m2g
    mopeka1_quality = m1q
    mopeka2_quality = m2q
    mopeka1_has_reading = True
    mopeka2_has_reading = True
    mopeka1_observed_at = _source_observation_epoch(m1_observed_at)
    mopeka2_observed_at = _source_observation_epoch(m2_observed_at)
    mopeka_enabled = True
    mopeka_connected = True
    print(f"Mopeka applied: front={m1g:.0f} back={m2g:.0f} q={m1q}/{m2q}", flush=True)
    update_mopeka_display()


def _apply_mopeka_sensor(
    index,
    gallons,
    quality,
    observed_at=None,
    level_mm=None,
    level_in=None,
    identity_token=None,
):
    """Apply one independently observed tank without filling the other with zero."""
    global mopeka1_gallons, mopeka2_gallons, mopeka1_quality, mopeka2_quality
    global mopeka1_level_mm, mopeka2_level_mm, mopeka1_level_in, mopeka2_level_in
    global mopeka1_has_reading, mopeka2_has_reading
    global mopeka1_observed_at, mopeka2_observed_at
    global mopeka_connected, mopeka_enabled
    if not _accept_trailer_sensor_identity(identity_token):
        return
    normalized_observed_at = _source_observation_epoch(observed_at)
    if index == 1:
        mopeka1_gallons = gallons
        mopeka1_quality = quality
        mopeka1_has_reading = True
        mopeka1_observed_at = normalized_observed_at
        if level_mm is not None:
            mopeka1_level_mm = level_mm
        if level_in is not None:
            mopeka1_level_in = level_in
    elif index == 2:
        mopeka2_gallons = gallons
        mopeka2_quality = quality
        mopeka2_has_reading = True
        mopeka2_observed_at = normalized_observed_at
        if level_mm is not None:
            mopeka2_level_mm = level_mm
        if level_in is not None:
            mopeka2_level_in = level_in
    else:
        return
    mopeka_enabled = True
    mopeka_connected = True
    print(
        f"Mopeka sensor {index} applied: gallons={gallons:.1f} q={quality}",
        flush=True,
    )
    update_mopeka_display()


def _apply_mopeka_raw(m1mm, m2mm, m1in, m2in):
    global mopeka1_level_mm, mopeka2_level_mm, mopeka1_level_in, mopeka2_level_in
    mopeka1_level_mm = m1mm
    mopeka2_level_mm = m2mm
    mopeka1_level_in = m1in
    mopeka2_level_in = m2in


def _apply_bms(soc, voltage, observed_at=None, identity_token=None):
    global bms_soc, bms_voltage, bms_has_reading, bms_observed_at
    if not _accept_trailer_sensor_identity(identity_token):
        return
    bms_soc = soc
    bms_voltage = voltage
    bms_has_reading = True
    bms_observed_at = _source_observation_epoch(observed_at)
    update_bms_display()


def _mopeka_offline():
    """Mark mopeka sensors as offline and update display"""
    global mopeka_connected
    mopeka_connected = False
    print("Mopeka offline", flush=True)
    update_mopeka_display()


def _mopeka_disabled():
    """Mark mopeka as intentionally disabled for this box."""
    global mopeka_connected, mopeka_enabled
    mopeka_enabled = False
    mopeka_connected = False
    print("Mopeka disabled", flush=True)
    update_mopeka_display()


def update_mopeka_display():
    """Draw Mopeka tank levels in top-right corner of screen"""
    canvas.delete("mopeka_display")

    if current_mode == "mix":
        refresh_batch_mix_tank_levels()
        return

    if not mopeka_enabled:
        canvas.delete("batchmix_tanks")
        return
    
    width = _canvas_width()
    x = width - 20  # 20px from right edge
    font = ("Helvetica", 72, "bold")
    
    if not mopeka_connected:
        canvas.create_text(x, 40, text="Tanks: No Signal", font=font,
                          fill="#ff0000", anchor="ne", tags="mopeka_display")
        return
    
    # Front tank - top right
    color1 = _mopeka_quality_color(mopeka1_quality)
    label1 = f"Front: {mopeka1_gallons:.0f}"
    canvas.create_text(x, 40, text=label1, font=font,
                      fill=color1, anchor="ne", tags="mopeka_display")
    
    # Back tank - below front
    color2 = _mopeka_quality_color(mopeka2_quality)
    label2 = f"Back: {mopeka2_gallons:.0f}"
    canvas.create_text(x, 110, text=label2, font=font,
                      fill=color2, anchor="ne", tags="mopeka_display")

def draw_requested_number(text, color="red"):
    """Draw the requested number with white outline on full-screen canvas"""
    global batch_mix_layout_active, _last_requested_text, _last_requested_color

    # If batch mix layout is active, use different positioning
    if batch_mix_layout_active:
        redraw_numbers_for_batch_mix()
        return

    # Only redraw if value or color changed (prevents flicker)
    if text == _last_requested_text and color == _last_requested_color:
        return

    _last_requested_text = text
    _last_requested_color = color

    # Delete old requested number
    canvas.delete("requested")

    # Position: centered horizontally, 20% from top
    x = _canvas_width() // 2
    y = int(_canvas_height() * 0.28) + 24
    font = ("Helvetica", 220, "bold")

    # Draw white outline (8 positions around the text)
    for dx, dy in [(-5,-5), (-5,0), (-5,5), (0,-5), (0,5), (5,-5), (5,0), (5,5)]:
        canvas.create_text(x+dx, y+dy, text=text, font=font, fill="white", tags="requested")

    # Draw text with specified color on top
    canvas.create_text(x, y, text=text, font=font, fill=color, tags="requested")

# Draw text labels on canvas (centered)
_sync_canvas_geometry()
center_x = _canvas_width() // 2
height = _canvas_height()

canvas.create_text(center_x, int(height * 0.08), text="Requested Gallons:", font=("Helvetica", 36, "bold"),
                  fill="white", tags="labels")

# Draw initial requested value
draw_requested_number(f"{config.REQUESTED_GALLONS:.0f}", "red")

canvas.create_text(center_x, int(height * 0.45), text="Actual Gallons:", font=("Helvetica", 36, "bold"),
                  fill="white", tags="labels")

def draw_actual_number(text, color="red"):
    """Draw the actual number with white outline on full-screen canvas"""
    global batch_mix_layout_active, _last_actual_text, _last_actual_color

    # If batch mix layout is active, use different positioning
    if batch_mix_layout_active:
        redraw_numbers_for_batch_mix()
        return

    # Only redraw if value or color changed (prevents flicker)
    if text == _last_actual_text and color == _last_actual_color:
        return

    _last_actual_text = text
    _last_actual_color = color

    # Delete old actual number
    canvas.delete("actual")

    # Position: centered horizontally, 68% from top
    x = _canvas_width() // 2
    y = int(_canvas_height() * 0.68)
    font = ("Helvetica", 310, "bold")

    # Draw white outline (8 positions around the text)
    for dx, dy in [(-6,-6), (-6,0), (-6,6), (0,-6), (0,6), (6,-6), (6,0), (6,6)]:
        canvas.create_text(x+dx, y+dy, text=text, font=font, fill="white", tags="actual")

    # Draw text with specified color on top
    canvas.create_text(x, y, text=text, font=font, fill=color, tags="actual")

# Draw initial value
draw_actual_number("0.0", "red")

# Draw initial mopeka state (no signal)
root.after(1000, update_mopeka_display)

# Status Label (for connection errors)
status_label = ttk.Label(root, text="", font=("Helvetica", 24),
                        foreground="yellow", background="black")
# status_label.pack(pady=2)

# Flow Meter Disconnected Warning (flashing)
flowmeter_disconnected_label = ttk.Label(root, text="FLOW METER\nDISCONNECTED",
                                         font=("Helvetica", 60, "bold"),
                                         foreground="red", background="black")
# flowmeter_disconnected_label.pack(pady=2)
# flowmeter_disconnected_label.pack_forget()

# Switch Box Disconnected Label (shown when heartbeat times out)
switchbox_disconnected_label = ttk.Label(root, text="SWITCH BOX\nDISCONNECTED",
                                         font=("Helvetica", 60, "bold"),
                                         foreground="red", background="black")
# switchbox_disconnected_label.pack(pady=2)
# switchbox_disconnected_label.pack_forget()

# Warning Label (flashing)
warning_label = ttk.Label(root, text="OVER TARGET!", font=("Helvetica", 72, "bold"),
                          foreground="red", background="black")
# warning_label.pack(pady=2)
# warning_label.pack_forget()

# Manual Mode Label (flashing when override is active)
manual_label = ttk.Label(root, text="MANUAL", font=("Helvetica", 90, "bold"),
                         foreground="orange", background="black")
# manual_label.pack(pady=2)
# manual_label.pack_forget()

# Mix Mode Indicator Label (shown in top-left corner when in mix mode)
mode_indicator_label = tk.Label(root, text="MIX", font=("Helvetica", 38, "bold"),
                                foreground="cyan", background="black",
                                padx=14, pady=4, bd=0)
# mode_indicator_label initially hidden, shown via place() when in mix mode

# Thumbs up animated GIF support
thumbs_up_frames = []
thumbs_up_frames_by_color = {}
thumbs_up_frame_index = [0]  # Use list for mutable reference
thumbs_up_label = None
thumbs_up_animation_id = None
thumbs_up_visible = False
thumbs_up_current_color = "green"
THUMBS_UP_RELX = 0.88
THUMBS_UP_RELY = 0.25
THUMBS_UP_TINTS = {
    "green": (0, 190, 0),
    "red": (230, 0, 0),
}

def tint_thumbs_up_image(image, rgb):
    """Tint a transparent thumbs-up image while keeping its shading and alpha."""
    rgba = image.convert("RGBA")
    alpha = rgba.getchannel("A")
    shade = rgba.convert("L")
    red, green, blue = rgb
    channels = [
        shade.point(lambda value, base=base: int(base * (0.45 + 0.55 * (value / 255.0))))
        for base in (red, green, blue)
    ]
    return Image.merge("RGBA", (*channels, alpha))


def set_thumbs_up_color(color):
    """Switch the thumbs-up image/text to match the gallon display color."""
    global thumbs_up_frames, thumbs_up_current_color

    color = "red" if color == "red" else "green"
    if color == thumbs_up_current_color and thumbs_up_frames:
        return

    thumbs_up_current_color = color
    if thumbs_up_frames_by_color:
        thumbs_up_frames = thumbs_up_frames_by_color.get(color, thumbs_up_frames)
        thumbs_up_frame_index[0] = min(thumbs_up_frame_index[0], max(len(thumbs_up_frames) - 1, 0))
        if thumbs_up_label and thumbs_up_frames:
            thumbs_up_label.config(image=thumbs_up_frames[thumbs_up_frame_index[0]])
    elif thumbs_up_label:
        thumbs_up_label.config(foreground=color)


def show_thumbs_up(actual_gallons=None):
    """Show thumbs-up with the same red/green state as the gallon text."""
    if not thumbs_up_label or batch_mix_layout_active:
        return
    set_thumbs_up_color(target_display_color(actual_gallons))
    thumbs_up_label.place(relx=THUMBS_UP_RELX, rely=THUMBS_UP_RELY, anchor="n")
    _set_thumbs_up_visible(True)

def load_thumbs_up_gif():
    """Load thumbs up image (PNG or GIF) for display"""
    global thumbs_up_frames, thumbs_up_frames_by_color, thumbs_up_label

    script_dir = os.path.dirname(os.path.abspath(__file__))
    png_candidates = [
        os.path.join(script_dir, "thumbs_up.png"),
        "/home/pi/thumbs_up.png",
    ]
    gif_candidates = [
        os.path.join(script_dir, "thumbs_up.gif"),
        "/home/pi/thumbs_up.gif",
    ]
    
    try:
        png_path = next((path for path in png_candidates if os.path.exists(path)), None)
        gif_path = next((path for path in gif_candidates if os.path.exists(path)), None)
        base_frames = []

        # Try PNG first
        if png_path:
            img = Image.open(png_path)
            # Resize to fit nicely on screen
            img = img.resize((533, 533), Image.Resampling.LANCZOS)
            base_frames = [img]
            print(f"Loaded thumbs up from PNG: {png_path}")
        elif gif_path:
            img = Image.open(gif_path)
            # Extract all frames from GIF
            try:
                while True:
                    frame = img.copy()
                    frame = frame.resize((533, 533), Image.Resampling.LANCZOS)
                    base_frames.append(frame)
                    img.seek(img.tell() + 1)
            except EOFError:
                pass
            print(f"Loaded {len(base_frames)} frames from thumbs up GIF")
        else:
            base_frames = []

        thumbs_up_frames_by_color = (
            {
                color: [ImageTk.PhotoImage(tint_thumbs_up_image(frame, rgb)) for frame in base_frames]
                for color, rgb in THUMBS_UP_TINTS.items()
            }
            if base_frames else {}
        )
        thumbs_up_frames = thumbs_up_frames_by_color.get(thumbs_up_current_color, [])
        
        # Create label
        if thumbs_up_frames:
            thumbs_up_label = tk.Label(root, image=thumbs_up_frames[0], bg="black")
        else:
            thumbs_up_label = tk.Label(root, text="👍", font=("Helvetica", 400, "bold"),
                                       foreground="green", background="black")
    except Exception as e:
        print(f"Could not load thumbs up image: {e}")
        thumbs_up_label = tk.Label(root, text="👍", font=("Helvetica", 400, "bold"),
                                   foreground="green", background="black")


def _set_thumbs_up_visible(visible):
    """Track whether the thumbs-up indicator is currently visible."""
    global thumbs_up_visible
    thumbs_up_visible = bool(visible)


def animate_thumbs_up():
    """Animate the thumbs up GIF"""
    global thumbs_up_animation_id
    
    if thumbs_up_frames and thumbs_up_label:
        thumbs_up_frame_index[0] = (thumbs_up_frame_index[0] + 1) % len(thumbs_up_frames)
        thumbs_up_label.config(image=thumbs_up_frames[thumbs_up_frame_index[0]])
        thumbs_up_animation_id = root.after(100, animate_thumbs_up)  # 10 FPS

# Load the GIF on startup
load_thumbs_up_gif()

# ---------------------------------------------------------------------------
# Display-chain self-healing.
#
# update_dashboard() is the box display's only redraw driver: a self-
# rescheduling Tk `after` chain. Before this guard, one exception anywhere in
# the ~500-line tick (field case: an unguarded log write on a full/read-only
# SD card) silently killed the chain forever - screen frozen while the flow-
# control thread and the :9999 listener kept serving live data to BLE/WiFi
# clients ("display dead but the app still shows flow"). Restarting the
# service is NOT an acceptable fix: it blanks the screen and drops pending-
# fill state mid-fill. Instead the chain heals in place:
#   1. Every tick is wrapped; the reschedule lives in `finally`, so a bad
#      tick costs one frame, not the display.
#   2. The tick stamps a heartbeat; the (fully guarded, always-alive) flow-
#      control thread re-kicks the chain via the Tk event queue if the
#      heartbeat ever goes stale - no restart, crews see at most a blip.
#   3. The heartbeat age is exported in STATE_JSON (display_tick_age_s) so a
#      wedged mainloop (the one case in-process code can't heal) is at least
#      visible remotely.
# ---------------------------------------------------------------------------
DISPLAY_TICK_STALE_SECONDS = 5.0
DISPLAY_REVIVE_MIN_SPACING_SECONDS = 10.0
TICK_ERROR_LOG = "/home/pi/dashboard_tick_errors.log"

last_dashboard_tick_at = time.time()
_dashboard_chain_started = False
_dashboard_after_id = None
_dashboard_tick_failures = 0
_dashboard_tick_last_error_at = 0.0
_dashboard_revives = 0
_display_revive_last_kick_at = 0.0


def _log_tick_error(text):
    """Best-effort tick-failure record; must never raise (the failing tick
    may itself be a disk problem) and never block."""
    line = f"{time.strftime('%Y-%m-%d %H:%M:%S')} - {text}\n"
    try:
        print(line.rstrip("\n"), flush=True)
    except Exception:
        pass
    try:
        # 5MB cap so a permanent fault can't grow this file unbounded even
        # if logrotate isn't running.
        if not (os.path.exists(TICK_ERROR_LOG) and os.path.getsize(TICK_ERROR_LOG) > 5_000_000):
            with open(TICK_ERROR_LOG, "a") as f:
                f.write(line)
    except Exception:
        pass


def update_dashboard():
    """Run one guarded display tick; the redraw chain must never die."""
    global _dashboard_after_id, _dashboard_tick_failures, _dashboard_tick_last_error_at
    global last_dashboard_tick_at, _dashboard_chain_started

    _dashboard_chain_started = True
    try:
        _update_dashboard_tick()
        _dashboard_tick_failures = 0
    except Exception:
        _dashboard_tick_failures += 1
        now = time.time()
        if now - _dashboard_tick_last_error_at > 30.0:
            _dashboard_tick_last_error_at = now
            _log_tick_error(
                f"dashboard tick failed ({_dashboard_tick_failures} consecutive)\n"
                + traceback.format_exc()
            )
    finally:
        last_dashboard_tick_at = time.time()
        try:
            _dashboard_after_id = root.after(config.UPDATE_INTERVAL, update_dashboard)
        except Exception:
            # Tk is tearing down (app exit); nothing left to keep alive.
            _dashboard_after_id = None


def _revive_dashboard_chain():
    """Runs ON the Tk thread. Restart a truly dead redraw chain in place.

    Freshness is re-checked here (not just in the flow-control thread) so a
    queued revive can never double-schedule a chain that is actually alive;
    any pending zombie tick is cancelled before the fresh one starts.
    """
    global _dashboard_revives

    if time.time() - last_dashboard_tick_at <= DISPLAY_TICK_STALE_SECONDS:
        return
    try:
        if _dashboard_after_id is not None:
            root.after_cancel(_dashboard_after_id)
    except Exception:
        pass
    _dashboard_revives += 1
    _log_tick_error(f"display chain revived in place (revive #{_dashboard_revives})")
    update_dashboard()


def _maybe_revive_display_chain():
    """Called from the flow-control thread each loop: queue an in-place
    revive when the display heartbeat goes stale. root.after is the same
    cross-thread mechanism the serial/socket listeners already use. Must
    never raise (it runs inside the safety-critical control loop)."""
    global _display_revive_last_kick_at

    try:
        if not _dashboard_chain_started:
            return
        now = time.time()
        if now - last_dashboard_tick_at <= DISPLAY_TICK_STALE_SECONDS:
            return
        if now - _display_revive_last_kick_at <= DISPLAY_REVIVE_MIN_SPACING_SECONDS:
            return
        _display_revive_last_kick_at = now
        log_flow_control(
            f"display_chain_stale | age={now - last_dashboard_tick_at:.1f}s | revive_kick"
        )
        root.after(0, _revive_dashboard_chain)
    except Exception:
        pass


def _update_dashboard_tick():
    """Update the dashboard with current flow meter readings"""
    global last_alert_triggered, auto_shutoff_latched, override_mode, was_flowing, colors_are_green, heartbeat_disconnected, override_enabled_time
    global new_fill_flow_started_at, new_fill_last_fresh_at, new_fill_cycle_cleared
    global pending_fill_gallons, pending_fill_requested, pending_fill_shutoff_type
    global pending_fill_flow_gpm, pending_fill_trigger_threshold, pending_fill_temp_f
    global pending_fill_stop_to_thumb_start_at
    global pending_fill_flow_started_at, pending_fill_flow_ended_at, current_fill_flow_started_at
    global last_flowing_rate_l_per_s
    global last_trigger_flow_gpm, last_trigger_threshold, last_trigger_actual
    global last_trigger_predicted_actual, last_trigger_loop_dt_ms
    global last_pump_stop_relay_activated_at
    global flow_cycle_counter, calibration_state, last_status_text, last_daily_total_text, last_daily_total_mode

    control_active = flow_control_active()

    if control_active:
        actual = get_cached_actual_gallons()
    else:
        actual = read_flow_meter()

    color = target_display_color(actual)
    if thumbs_up_visible:
        set_thumbs_up_color(color)

    draw_actual_number(f"{actual:.1f}", color)

    # Update requested gallons number
    draw_requested_number(f"{requested_gallons:.0f}", color)

    # Check if flow meter has timed out (no successful reads in X seconds)
    flow_meter_disconnected = (time.time() - last_successful_read_time) > config.FLOW_METER_TIMEOUT
    update_flow_meter_fault_hold(flow_meter_disconnected)

    # Check if heartbeat has timed out (no OK message in 11 seconds)
    heartbeat_timeout = (time.time() - last_heartbeat_time) > 11
    if heartbeat_timeout and not heartbeat_disconnected:
        heartbeat_disconnected = True
        msg = "Heartbeat timeout - Switch box disconnected"
        print(msg)
        log_serial_debug(msg)
    elif not heartbeat_timeout and heartbeat_disconnected:
        heartbeat_disconnected = False
        msg = "Heartbeat restored - Switch box reconnected"
        print(msg)
        log_serial_debug(msg)

    # Detect flow state
    is_flowing = last_flow_rate >= config.FLOW_STOPPED_THRESHOLD
    now = time.time()

    # Field evidence for sub-threshold flow: with the 4 GPM event threshold a
    # post-shutoff dribble creates no edges and would otherwise leave no trace
    # in the journal - one summary line when it decays back to zero confirms
    # dribbles are real (the TR12 flow-window theory) without log spam. Ramps
    # into a real fill (ends with is_flowing True) are normal and not logged.
    global _sub_threshold_flow_started_at, _sub_threshold_flow_peak_lps
    global _sub_threshold_flow_start_gal, _sub_threshold_flow_last_log_at
    if not is_flowing and last_flow_rate > config.FLOW_METER_ZERO_THRESHOLD:
        if _sub_threshold_flow_started_at == 0.0:
            _sub_threshold_flow_started_at = now
            _sub_threshold_flow_start_gal = actual
            _sub_threshold_flow_peak_lps = 0.0
        _sub_threshold_flow_peak_lps = max(_sub_threshold_flow_peak_lps, last_flow_rate)
    elif _sub_threshold_flow_started_at > 0.0:
        if not is_flowing and now - _sub_threshold_flow_last_log_at >= 30.0:
            _sub_threshold_flow_last_log_at = now
            print(
                'Sub-threshold flow ended (no fill event): '
                f'peak {_sub_threshold_flow_peak_lps * config.LITERS_PER_SEC_TO_GPM:.1f} GPM, '
                f'+{actual - _sub_threshold_flow_start_gal:.3f} gal over '
                f'{now - _sub_threshold_flow_started_at:.1f}s',
                flush=True,
            )
        _sub_threshold_flow_started_at = 0.0
        _sub_threshold_flow_peak_lps = 0.0
        _sub_threshold_flow_start_gal = 0.0
    latest_fresh_age = now - last_fresh_flow_read_time if last_fresh_flow_read_time else float("inf")
    fresh_grace_seconds = config.NEW_FILL_CYCLE_FRESH_GRACE_SECONDS
    has_recent_fresh_flow = latest_fresh_age <= fresh_grace_seconds
    is_recent_fresh_new_fill_flowing = (
        has_recent_fresh_flow
        and not flow_meter_disconnected
        and not connection_error
        and last_flow_rate >= config.NEW_FILL_CYCLE_THRESHOLD
    )
    if is_recent_fresh_new_fill_flowing:
        if new_fill_flow_started_at is None:
            new_fill_flow_started_at = last_fresh_flow_read_time
        new_fill_last_fresh_at = last_fresh_flow_read_time
    elif (
        flow_meter_disconnected
        or connection_error
        or last_flow_rate < config.NEW_FILL_CYCLE_THRESHOLD
        or (
            new_fill_last_fresh_at is not None
            and now - new_fill_last_fresh_at > fresh_grace_seconds
        )
    ):
        new_fill_flow_started_at = None
        new_fill_last_fresh_at = None
        new_fill_cycle_cleared = False
    if is_flowing and not control_active and last_flow_read_was_fresh:
        recent_flow_rates_l_per_s.append(last_flow_rate)
        last_flowing_rate_l_per_s = last_flow_rate

    calibration_waiting_for_fill = (
        calibration_mode
        and calibration_state
        and calibration_state.get("phase") == "wait_for_fill"
        and calibration_state.get("flow_started")
    )

    # Store fill data when flow stops (don't record yet - wait for thumbs up)
    if was_flowing and not is_flowing:
        if calibration_waiting_for_fill and (
            actual - calibration_state.get("flow_start_actual", actual)
        ) < CALIBRATION_MIN_FILL_GALLONS:
            # Meter blip / post-shutoff dribble, not a fill (seen starting the
            # settle countdown before the pump ever ran). Keep waiting.
            calibration_state["flow_started"] = False
            print(
                f"Calibration: ignoring flow blip of "
                f"{actual - calibration_state.get('flow_start_actual', actual):.2f} gal",
                flush=True,
            )
        elif calibration_waiting_for_fill:
            calibration_state["last_step_actual"] = actual
            calibration_state["phase"] = "settling"
            calibration_state["settle_deadline"] = time.time() + 120
            calibration_state["flow_started"] = False
            if thumbs_up_label:
                thumbs_up_label.place_forget()
                _set_thumbs_up_visible(False)
            pending_fill_gallons = 0.0
            pending_fill_requested = 0.0
            pending_fill_shutoff_type = ""
            pending_fill_flow_gpm = 0.0
            pending_fill_trigger_threshold = 0.0
            pending_fill_temp_f = None
            pending_fill_stop_to_thumb_start_at = 0.0
            pending_fill_flow_started_at = 0.0
            pending_fill_flow_ended_at = 0.0
            last_trigger_flow_gpm = 0.0
            last_trigger_threshold = 0.0
            last_trigger_actual = 0.0
            last_pump_stop_relay_activated_at = 0.0
            recent_flow_rates_l_per_s.clear()
            _refresh_calibration_window()
        elif _is_fill_flow_continuation(
            pending_fill_gallons,
            pending_fill_requested,
            pending_fill_flow_started_at,
            requested_gallons,
            actual,
        ):
            # Post-shutoff dribble / tiny top-off of the staged fill: take the
            # extra volume and extend the flow end, keep everything else from
            # the real fill (window start, shutoff type, FlowAtStop,
            # stop-to-thumb). See _is_fill_flow_continuation.
            print(
                "Fill dribble folded into pending fill - "
                f"Actual: {actual:.3f} (+{actual - pending_fill_gallons:.3f} gal, "
                f"{now - pending_fill_flow_ended_at:.1f}s after flow end, "
                f"target {pending_fill_requested:.1f} gal, flow window kept)"
            )
            pending_fill_gallons = actual
            pending_fill_flow_ended_at = now
        else:
            # Determine if shutoff was automatic or manual
            shutoff_type = "Auto" if auto_shutoff_latched else "Manual"

            if shutoff_type == "Auto" and last_trigger_flow_gpm > 0:
                pending_fill_flow_gpm = last_trigger_flow_gpm
                pending_fill_trigger_threshold = last_trigger_threshold
            else:
                smoothed_stop_flow_l_per_s = get_smoothed_flow_rate()
                pending_fill_flow_gpm = smoothed_stop_flow_l_per_s * config.LITERS_PER_SEC_TO_GPM
                pending_fill_trigger_threshold = calculate_trigger_threshold(smoothed_stop_flow_l_per_s)

            # Store pending fill data (will be recorded when thumbs up is pressed)
            pending_fill_gallons = actual
            pending_fill_requested = requested_gallons
            pending_fill_shutoff_type = shutoff_type
            pending_fill_temp_f = last_flow_meter_temp_f
            pending_fill_stop_to_thumb_start_at = last_pump_stop_relay_activated_at
            # Flow window for this fill: start was captured at the flow rising edge
            # (current_fill_flow_started_at); end is now, the moment flow stopped.
            # If the start was never observed (e.g. box restarted mid-fill) it stays
            # 0.0 and is omitted from the log so the app flags the record.
            pending_fill_flow_started_at = current_fill_flow_started_at
            pending_fill_flow_ended_at = now

            print(
                f"Fill complete - Requested: {requested_gallons:.3f}, Actual: {actual:.3f}, "
                f"Diff: {actual - requested_gallons:+.3f}, Type: {shutoff_type}, "
                f"FlowAtStop: {pending_fill_flow_gpm:.1f} GPM, Threshold: {pending_fill_trigger_threshold:.3f}"
            )
            print(f"Waiting for thumbs up button to record fill...")

            # NOTE: Do NOT hide thumbs up when flow stops - keep it visible so user can press it
            # Thumbs up will be hidden after button is pressed or when new fill starts

    # Reset colors when new fill cycle starts
    if not was_flowing and is_flowing:
        colors_are_green = False
        new_fill_cycle_cleared = False
        # Flow just started for this fill segment — stamp the start time so it can
        # be recorded alongside the flow-end time when the fill completes.
        current_fill_flow_started_at = now
        if calibration_mode and calibration_state and calibration_state.get("phase") == "wait_for_fill":
            calibration_state["flow_started"] = True
            calibration_state["flow_start_actual"] = actual
            _refresh_calibration_window()
        elif calibration_mode and calibration_state and calibration_state.get("phase") == "settling":
            # Pump (re)started during the settle wait — the settle is void, this
            # is the real fill. Fall back to waiting for it to finish so the
            # 2-minute clock starts from the true pump stop.
            calibration_state["phase"] = "wait_for_fill"
            calibration_state["flow_started"] = True
            calibration_state["flow_start_actual"] = actual
            calibration_state["settle_deadline"] = None
            print("Calibration: flow started during settle - back to wait_for_fill", flush=True)
            _refresh_calibration_window()
        if not control_active:
            flow_cycle_counter += 1
            last_alert_triggered = False
            auto_shutoff_latched = False
            last_trigger_flow_gpm = 0.0
            last_trigger_threshold = 0.0
            last_trigger_actual = 0.0
            last_trigger_predicted_actual = 0.0
            last_trigger_loop_dt_ms = 0.0
            last_pump_stop_relay_activated_at = 0.0
            recent_flow_rates_l_per_s.clear()
        print("New fill cycle started - colors reset to red, auto-shutoff state cleared")

    # Hide thumbs up and clear pending fill only after sustained high flow.
    if (
        new_fill_flow_started_at is not None
        and new_fill_last_fresh_at is not None
        and not new_fill_cycle_cleared
        and now - new_fill_flow_started_at >= config.NEW_FILL_CYCLE_HOLD_SECONDS
        and now - new_fill_last_fresh_at <= config.NEW_FILL_CYCLE_FRESH_GRACE_SECONDS
    ):
        if thumbs_up_label:
            thumbs_up_label.place_forget()
            _set_thumbs_up_visible(False)
        pending_fill_gallons = 0.0
        pending_fill_requested = 0.0
        pending_fill_shutoff_type = ""
        pending_fill_flow_gpm = 0.0
        pending_fill_trigger_threshold = 0.0
        pending_fill_temp_f = None
        pending_fill_stop_to_thumb_start_at = 0.0
        pending_fill_flow_started_at = 0.0
        pending_fill_flow_ended_at = 0.0
        new_fill_cycle_cleared = True
        print("Sustained high-flow fill cycle detected - thumbs up hidden, pending fill cleared")

    if calibration_mode and calibration_state and calibration_state.get("phase") == "settling":
        if time.time() >= calibration_state.get("settle_deadline", time.time()):
            calibration_state["phase"] = "review"
            calibration_state["reading"] = _selected_tank_reading()
        _refresh_calibration_window()

    # Update flow state for next cycle
    was_flowing = is_flowing

    # Auto-disable override after 1 minute if no flow (only if flow meter is connected)
    if override_mode and not flow_meter_disconnected:
        if last_flow_rate < config.FLOW_STOPPED_THRESHOLD:
            # No flow detected - check if override has been enabled for more than 60 seconds
            time_since_override_enabled = time.time() - override_enabled_time
            if time_since_override_enabled > 60:
                override_mode = False
                print(f"Override auto-disabled: no flow for {time_since_override_enabled:.0f} seconds (> 60s limit)")
        else:
            # Flow detected - reset the timer so override can stay active
            override_enabled_time = time.time()
    # If flow meter is disconnected, override stays on indefinitely (no auto-disable)

    # Calculate dynamic trigger threshold based on current flow rate
    smoothed_flow_rate_l_per_s = get_smoothed_flow_rate()
    flow_rate_gpm = smoothed_flow_rate_l_per_s * config.LITERS_PER_SEC_TO_GPM
    display_flow_rate_gpm = flow_rate_gpm if is_flowing else 0.0
    trigger_threshold = calculate_trigger_threshold(smoothed_flow_rate_l_per_s)

    # Auto-alert: Trigger GPIO 27 based on flow-adjusted threshold (once per cycle)
    # Only if override mode is OFF and flow meter is connected
    if is_flowing and not override_mode and not flow_meter_disconnected and actual >= requested_gallons - trigger_threshold and not last_alert_triggered:
        last_alert_triggered = True
        auto_shutoff_latched = True
        last_trigger_flow_gpm = flow_rate_gpm
        last_trigger_threshold = trigger_threshold
        last_trigger_actual = actual
        print(f"Auto-alert: Flow={flow_rate_gpm:.1f} GPM, threshold={trigger_threshold:.2f}gal, triggering relay for {config.AUTO_ALERT_DURATION}s")
        start_pump_stop_thread(config.AUTO_ALERT_DURATION)
    startup_iol_warning_suppressed = _suppress_startup_iol_warning(flow_meter_disconnected)

    # Update status label
    status_parts = []
    if startup_iol_warning_suppressed:
        status_parts.append("IOL: Starting")
    elif pump_stop_fault_hold_active:
        status_parts.append(f"IOL: {pump_stop_fault_hold_reason[:30]}")
    elif connection_error:
        status_parts.append(f"IOL: {error_message[:30]}")
    else:
        status_parts.append("IOL: Connected")

    if serial_connected:
        # Show Connected or Disconnected based on heartbeat status
        if heartbeat_disconnected:
            status_parts.append("Serial: Disconnected")
        else:
            status_parts.append("Serial: Connected")
    else:
        status_parts.append("Serial: Disconnected")

    if override_mode:
        status_parts.append("OVERRIDE: ON")

    # Draw status text only when it changes.
    status_text = " | ".join(status_parts)
    if status_text != last_status_text:
        canvas.delete("status")
        if status_text:
            canvas.create_text(_canvas_width() // 2, _canvas_height() - 20, text=status_text,
                              font=("Helvetica", 20), fill="yellow", tags="status")
        last_status_text = status_text

    # Draw daily total only when the visible text changes.
    daily_total_text = f"Today:\n{daily_total:.1f} gal" if current_mode != "mix" else ""
    if daily_total_text != last_daily_total_text or current_mode != last_daily_total_mode:
        canvas.delete("daily_total")
        if daily_total_text:
            canvas.create_text(10, _canvas_height() - 10, text=daily_total_text,
                              font=("Helvetica", 72, "bold"), fill="cyan", anchor="sw", tags="daily_total")
        last_daily_total_text = daily_total_text
        last_daily_total_mode = current_mode
    signed_flow_gpm = last_flow_rate * config.LITERS_PER_SEC_TO_GPM
    if signed_flow_gpm < 0:
        update_flow_rate_display(signed_flow_gpm, alert=True)
    else:
        update_flow_rate_display(display_flow_rate_gpm)

    # Draw skull icons on sides when flow meter is disconnected (3 inches ~= 288pt at 96 DPI)
    # Pulse animation: size varies between 240pt and 288pt with 1-second cycle
    canvas.delete("skull_icons")
    if flow_meter_disconnected and not startup_iol_warning_suppressed:
        import math
        pulse = math.sin(time.time() * 2 * math.pi)  # -1 to 1, completes cycle every 1 second
        skull_size = int(264 + 24 * pulse)  # Varies from 240pt to 288pt

        # Left skull
        canvas.create_text(150, _canvas_height() // 2, text="☠",
                         font=("Helvetica", skull_size, "bold"), fill="red", tags="skull_icons")
        # Right skull
        canvas.create_text(_canvas_width() - 150, _canvas_height() // 2, text="☠",
                         font=("Helvetica", skull_size, "bold"), fill="red", tags="skull_icons")

    # Draw warnings on canvas - collect all active warnings and cycle through them
    canvas.delete("warning")
    canvas.delete("caution_blocks")  # Delete caution blocks from previous frame
    flow_meter_drift_alarm_active = (
        negative_totalizer_fault_active
        or negative_flow_fault_active
        or positive_drift_fault_active
    )

    # Special handling for override/caution mode - draw flashing red blocks with caution symbols
    if override_mode:
        # Railroad crossing alternating flash pattern (1 Hz - left side / right side alternate every 0.5s)
        phase = int(time.time() * 2) % 2  # 0 or 1

        # Block dimensions - larger to fit caution symbol
        block_width = 320
        block_height = 380

        # Calculate vertical positions for upper and lower blocks
        upper_y = int(_canvas_height() * 0.35)  # Upper blocks at 35% down screen
        lower_y = int(_canvas_height() * 0.65)  # Lower blocks at 65% down screen

        # Block positions (left and right sides)
        left_x = 50
        right_x = _canvas_width() - 50 - block_width

        # Draw LEFT side blocks when phase=0 (both upper and lower left)
        if phase == 0:
            # Upper left block
            canvas.create_rectangle(left_x, upper_y - block_height//2,
                                   left_x + block_width, upper_y + block_height//2,
                                   fill="red", outline="", tags="caution_blocks")
            canvas.create_text(left_x + block_width//2, upper_y,
                             text="⚠", font=("Helvetica", 180, "bold"),
                             fill="white", tags="caution_blocks")

            # Lower left block
            canvas.create_rectangle(left_x, lower_y - block_height//2,
                                   left_x + block_width, lower_y + block_height//2,
                                   fill="red", outline="", tags="caution_blocks")
            canvas.create_text(left_x + block_width//2, lower_y,
                             text="⚠", font=("Helvetica", 180, "bold"),
                             fill="white", tags="caution_blocks")

        # Draw RIGHT side blocks when phase=1 (both upper and lower right)
        else:
            # Upper right block
            canvas.create_rectangle(right_x, upper_y - block_height//2,
                                   right_x + block_width, upper_y + block_height//2,
                                   fill="red", outline="", tags="caution_blocks")
            canvas.create_text(right_x + block_width//2, upper_y,
                             text="⚠", font=("Helvetica", 180, "bold"),
                             fill="white", tags="caution_blocks")

            # Lower right block
            canvas.create_rectangle(right_x, lower_y - block_height//2,
                                   right_x + block_width, lower_y + block_height//2,
                                   fill="red", outline="", tags="caution_blocks")
            canvas.create_text(right_x + block_width//2, lower_y,
                             text="⚠", font=("Helvetica", 180, "bold"),
                             fill="white", tags="caution_blocks")

        # Draw "MANUAL" text at center bottom
        canvas.create_text(_canvas_width() // 2, int(_canvas_height() * 0.88),
                         text="MANUAL", font=("Helvetica", 90, "bold"),
                         fill="orange", tags="warning")
        if (
            pump_stop_fault_hold_active
            and not startup_iol_warning_suppressed
            and not flow_meter_drift_alarm_active
        ):
            canvas.create_text(_canvas_width() // 2, int(_canvas_height() * 0.15),
                             text=f"IO-LINK FAULT\n{pump_stop_fault_hold_reason}",
                             font=("Helvetica", 72, "bold"),
                             fill="red", tags="warning")

    else:
        # Build list of active warnings (in priority order) - only when NOT in override mode
        active_warnings = []

        if (
            pump_stop_fault_hold_active
            and not startup_iol_warning_suppressed
            and not flow_meter_drift_alarm_active
        ):
            active_warnings.append((
                f"IO-LINK FAULT\n{pump_stop_fault_hold_reason}",
                "Helvetica",
                72,
                "red",
            ))

        if heartbeat_disconnected:
            active_warnings.append(("SWITCH BOX\nDISCONNECTED", "Helvetica", 60, "red"))

        if flow_meter_disconnected and not startup_iol_warning_suppressed:
            active_warnings.append(("☠ FLOW METER ☠\nDISCONNECTED", "Helvetica", 60, "red"))

        if actual > requested_gallons + config.WARNING_THRESHOLD:
            active_warnings.append(("OVER TARGET!", "Helvetica", 72, "red"))

        # Display warnings - cycle through them if multiple exist
        if active_warnings:
            # Flash on/off at 2Hz (on for 0.5s, off for 0.5s)
            if int(time.time() * 2) % 2 == 0:
                # If multiple warnings, cycle through them every 3 seconds
                if len(active_warnings) > 1:
                    warning_index = int(time.time() / 3) % len(active_warnings)
                else:
                    warning_index = 0

                text, font_family, font_size, color = active_warnings[warning_index]
                canvas.create_text(_canvas_width() // 2, int(_canvas_height() * 0.88),
                                 text=text, font=(font_family, font_size, "bold"), fill=color, tags="warning")

    if relay_slowdown_alarm_active and relay_slowdown_alarm_visible:
        canvas.tag_raise("relay_slowdown_alarm")
    if (
        negative_totalizer_fault_active
        or negative_flow_fault_active
        or positive_drift_fault_active
    ) and negative_totalizer_alarm_visible:
        canvas.tag_raise("negative_totalizer_alarm")
    draw_physical_reset_safety_banner()


def _sim_send_command(line):
    """Drive the most common switch-box commands from the simulator panel."""
    global requested_gallons, serial_connected, override_mode, override_enabled_time
    global colors_are_green, fill_requested_gallons, mix_requested_gallons, current_mode
    global last_heartbeat_time

    line = str(line).strip()
    serial_connected = True

    if line == "OK":
        last_heartbeat_time = time.time()
        return

    if should_ignore_menu_ov_bounce(line, "Sim"):
        return

    if _sim_handle_confirmation_command(line):
        return

    if menu_mode:
        if line == "+1":
            menu_navigate_down()
        elif line == "-1":
            menu_navigate_up()
        elif line == "OV":
            arm_menu_ov_guard()
            menu_select()
        elif line == "PS":
            close_menu()
        return

    if line in ["+1", "-1", "+10", "-10"]:
        adjustment = int(line)
        if adjust_batch_mix_gallons(adjustment, "Sim"):
            return
        requested_gallons = max(0, requested_gallons + adjustment)
        colors_are_green = False
        if current_mode == "fill":
            fill_requested_gallons = requested_gallons
        else:
            mix_requested_gallons = requested_gallons
        save_mode_presets()
        draw_requested_number(f"{requested_gallons:.0f}", "red")
        update_batch_mix_overlay()
    elif line == "PS":
        handle_box_pump_stop_button("sim pump stop")
    elif line == "OV":
        if requested_gallons == 0:
            arm_menu_ov_guard()
            show_menu()
        else:
            override_mode = not override_mode
            if override_mode:
                override_enabled_time = time.time()
    elif line == "TU":
        handle_thumbs_up_press("sim TU")
    elif line in ("RST", "RESET"):
        force_flow_reset("sim reset")
    elif line == "MIX":
        switch_mode("mix")
    elif line == "FILL":
        switch_mode("fill")


def _sim_handle_confirmation_command(line):
    """Route simulator OV/PS through the same confirmation dialogs as serial."""
    if line not in {"OV", "PS"}:
        return False

    confirmation_pairs = [
        (exit_confirm_window, exit_confirm_handler, exit_cancel_handler),
        (reset_season_confirm_window, reset_season_confirm_handler, reset_season_cancel_handler),
        (reset_flow_curve_confirm_window, reset_flow_curve_confirm_handler, reset_flow_curve_cancel_handler),
        (accept_flow_curve_confirm_window, accept_flow_curve_confirm_handler, accept_flow_curve_cancel_handler),
    ]
    for window, confirm_handler, cancel_handler in confirmation_pairs:
        if window:
            handler = confirm_handler if line == "OV" else cancel_handler
            if handler:
                root.after(0, handler)
            return True

    if reminders_mode:
        if line == "OV":
            root.after(0, dismiss_reminders)
        return True

    return False


def _sim_bind_keyboard_shortcuts(panel):
    """Map simple keyboard keys to the simulated switch-box controls."""
    key_commands = {
        "p": "PS",
        "o": "OV",
        "Left": "-1",
        "Right": "+1",
    }

    def handle_key(event):
        key = event.keysym
        command = key_commands.get(key)
        if command is None:
            command = key_commands.get(getattr(event, "char", "").lower())
        if command:
            _sim_send_command(command)
            return "break"
        return None

    def bind_widget(widget):
        widget.bind("<KeyPress>", handle_key)
        for child in widget.winfo_children():
            bind_widget(child)

    bind_widget(root)
    bind_widget(panel)
    root.bind_all("<KeyPress>", handle_key)
    panel.focus_force()


def _create_sim_controls():
    """Create a small side window for driving the dashboard without Pi hardware."""
    global serial_connected, last_heartbeat_time

    serial_connected = True
    last_heartbeat_time = time.time()

    panel = tk.Toplevel(root)
    panel.title("BBB Simulator")
    panel.geometry("360x560")
    panel.configure(bg="#202020")

    status_var = tk.StringVar()
    flow_var = tk.DoubleVar(value=0)
    connected_var = tk.BooleanVar(value=True)
    switchbox_var = tk.BooleanVar(value=True)

    def set_flow_from_slider(_value=None):
        iolhat.set_flow_gpm(flow_var.get())

    def set_flow(value):
        flow_var.set(value)
        iolhat.set_flow_gpm(value)

    def set_iol_connected():
        iolhat.connected = connected_var.get()

    def set_switchbox_connected():
        global serial_connected, last_heartbeat_time
        serial_connected = switchbox_var.get()
        if serial_connected:
            last_heartbeat_time = time.time()

    def heartbeat_tick():
        if switchbox_var.get():
            _sim_send_command("OK")
        root.after(2500, heartbeat_tick)

    def update_status():
        actual_gal = iolhat.totalizer_liters * config.LITERS_TO_GALLONS
        actual_flow_gpm = iolhat.get_flow_gpm()
        status_var.set(
            f"Flow: {actual_flow_gpm:.0f} GPM | Actual: {actual_gal:.1f} gal | "
            f"IOL: {'on' if iolhat.connected else 'off'} | Switch: {'on' if switchbox_var.get() else 'off'}"
        )
        root.after(250, update_status)

    title = ttk.Label(panel, text="BBB Simulator", font=("Helvetica", 18, "bold"))
    title.pack(pady=(14, 6))

    status = ttk.Label(panel, textvariable=status_var, wraplength=320)
    status.pack(pady=(0, 12))

    ttk.Label(panel, text="Flow Rate").pack(anchor="w", padx=16)
    flow_scale = ttk.Scale(panel, from_=0, to=120, variable=flow_var, command=set_flow_from_slider)
    flow_scale.pack(fill="x", padx=16, pady=(2, 10))

    flow_buttons = ttk.Frame(panel)
    flow_buttons.pack(fill="x", padx=16, pady=(0, 12))
    for label, value in [("Stop", 0), ("45 GPM", 45), ("75 GPM", 75), ("100 GPM", 100)]:
        ttk.Button(flow_buttons, text=label, command=lambda v=value: set_flow(v)).pack(
            side="left", expand=True, fill="x", padx=2
        )

    cmd_frame = ttk.LabelFrame(panel, text="Switch Box")
    cmd_frame.pack(fill="x", padx=16, pady=8)
    for row in [("+1", "-1", "+10", "-10"), ("OV", "PS", "TU", "RST"), ("FILL", "MIX")]:
        row_frame = ttk.Frame(cmd_frame)
        row_frame.pack(fill="x", padx=6, pady=4)
        for command in row:
            ttk.Button(row_frame, text=command, command=lambda c=command: _sim_send_command(c)).pack(
                side="left", expand=True, fill="x", padx=2
            )

    sensors = ttk.LabelFrame(panel, text="Sensors")
    sensors.pack(fill="x", padx=16, pady=8)
    ttk.Checkbutton(
        sensors,
        text="IO-Link connected",
        variable=connected_var,
        command=set_iol_connected,
    ).pack(anchor="w", padx=8, pady=4)
    ttk.Checkbutton(
        sensors,
        text="Switch box heartbeat",
        variable=switchbox_var,
        command=set_switchbox_connected,
    ).pack(anchor="w", padx=8, pady=4)
    ttk.Button(sensors, text="Reset Flow Totalizer", command=iolhat.reset_totalizer).pack(
        fill="x", padx=8, pady=4
    )
    ttk.Button(
        sensors,
        text="Tanks Good",
        command=lambda: (_apply_mopeka(325, 310, 3, 3), _apply_mopeka_raw(640, 610, 25.2, 24.0)),
    ).pack(fill="x", padx=8, pady=4)
    ttk.Button(sensors, text="Tanks Offline", command=_mopeka_offline).pack(fill="x", padx=8, pady=4)
    ttk.Button(sensors, text="Battery 84%", command=lambda: _apply_bms(84, 13.1)).pack(
        fill="x", padx=8, pady=4
    )

    ttk.Button(panel, text="Quit Simulator", command=root.destroy).pack(fill="x", padx=16, pady=16)

    heartbeat_tick()
    update_status()
    _sim_bind_keyboard_shortcuts(panel)


# Initialize GPIO and IO-Link and start serial listener
gpio_ok = initialize_gpio()
iol_ok = initialize_iol()

if gpio_ok:
    print(f"Starting dashboard (GPIO: OK, IOL: {'OK' if iol_ok else 'FAILED - display frozen'})")
else:
    print(f"Starting dashboard (GPIO: FAILED - no relay control, IOL: {'OK' if iol_ok else 'FAILED - display frozen'})")

# Load totals from files
load_totals()
load_last_load()
load_flow_curve_state()
today_str = time.strftime('%Y-%m-%d')
if last_reset_date != today_str:
    print(f"Date changed since last reset ({last_reset_date} -> {today_str}) - resetting daily total on startup")
    reset_daily_total()
print(f"Loaded totals - Daily: {daily_total:.2f}, Season: {season_total:.2f}")
print(f"Loaded last loads - {last_loads_gallons[:3]}")

# Load mode presets and set initial requested gallons
load_mode_presets()
if current_mode == 'fill':
    requested_gallons = fill_requested_gallons
else:
    requested_gallons = mix_requested_gallons
    # Show mode indicator if starting in mix mode
    if mode_indicator_label:
        place_mix_mode_indicator()
print(f"Loaded mode - Mode: {current_mode}, Requested: {requested_gallons}, Fill preset: {fill_requested_gallons}, Mix preset: {mix_requested_gallons}")

# Redraw requested gallons with the loaded value
draw_requested_number(f"{requested_gallons:.0f}", "red")
update_last_load_display()
update_bms_display()

if SIM_MODE:
    root.after(100, _create_sim_controls)
else:
    # Start serial listener in background thread (works without IOL)
    serial_thread = threading.Thread(target=serial_listener, daemon=True)
    serial_thread.start()

    # Start socket command listener in background thread (for BLE server communication)
    socket_thread = threading.Thread(target=socket_command_listener, daemon=True)
    socket_thread.start()

    # Start green button monitor in background thread
    green_button_thread = threading.Thread(target=green_button_monitor, daemon=True)
    green_button_thread.start()

start_flow_control_thread()

def daily_total_checker():
    """Background thread to check time and reset daily total at 1:00 AM"""
    global last_reset_date
    import datetime

    while True:
        try:
            current_time = datetime.datetime.now()
            current_hour = current_time.hour
            current_minute = current_time.minute
            current_date = current_time.strftime('%Y-%m-%d')

            # Check if it's 1:00 AM and we haven't reset today
            if current_hour == 1 and current_minute == 0:
                if current_date != last_reset_date:
                    print(f"It's 1:00 AM - resetting daily total for {current_date}")
                    reset_daily_total()
                # Sleep for 61 seconds to avoid re-triggering during the same minute
                time.sleep(61)
            else:
                # Check every 30 seconds
                time.sleep(30)
        except Exception as e:
            print(f"Error in daily_total_checker: {e}")
            time.sleep(60)

def daily_reminder_checker():
    """Background thread to check time and show reminders at 2 AM daily"""
    global last_reminder_date, reminders_mode

    while True:
        try:
            current_time = time.localtime()
            current_hour = current_time.tm_hour
            current_minute = current_time.tm_min
            current_date = time.strftime('%Y-%m-%d')

            # Check if it's 2 AM and we haven't shown reminders today
            if current_hour == 2 and current_minute == 0:
                if current_date != last_reminder_date and not reminders_mode:
                    print(f"It's 2 AM - showing daily reminders for {current_date}")
                    root.after(0, show_daily_reminders)
                # Sleep for 61 seconds to avoid re-triggering during the same minute
                time.sleep(61)
            else:
                # Check every 30 seconds
                time.sleep(30)
        except Exception as e:
            print(f"Error in daily_reminder_checker: {e}")
            time.sleep(60)

# Start daily total checker thread
total_checker_thread = threading.Thread(target=daily_total_checker, daemon=True)
total_checker_thread.start()

# Start daily reminder checker thread
reminder_thread = threading.Thread(target=daily_reminder_checker, daemon=True)
reminder_thread.start()

update_negative_totalizer_fault_flash()
update_relay_slowdown_alarm_flash()
update_dashboard()

if SIM_MODE:
    root.deiconify()
    root.lift()
    root.update_idletasks()

try:
    root.mainloop()
finally:
    flow_control_stop_event.set()
    if flow_control_thread and flow_control_thread.is_alive():
        flow_control_thread.join(timeout=1.0)
    # Cleanup GPIO on exit
    if GPIO_AVAILABLE:
        GPIO.cleanup()
