"""
state_translator.py
-------------------
Pure translation logic — no ROS2 or MQTT imports here.
Takes structured Python dicts / primitives and emits Nano commands
through NanoInterface.  Keeps all "business rules" in one place.
"""

import re
import logging
from typing import Optional
from robot_nano_bridge.nano_interface import (
    NanoInterface,
    LED_MODE_PATROL, LED_MODE_GUIDANCE, LED_MODE_DOCKING,


)

logger = logging.getLogger(__name__)

# ── Tuning constants ──────────────────────────────────────────────────────────
ANGULAR_TURN_THRESHOLD = 0.1   # rad/s  — below this → STOP (angular)
LINEAR_MOVE_THRESHOLD  = 0.02  # m/s    — below this → STOP (linear)

# command_id patterns → LED mode
_MODE_PATTERNS = [
    (re.compile(r"^PATROL_", re.IGNORECASE),   LED_MODE_PATROL),
    (re.compile(r"^DOCK_",   re.IGNORECASE),   LED_MODE_DOCKING),
    # anything else that has an alphanumeric ID → GUIDANCE
]

# Failure status keywords → Error mode (Red)
_ERROR_STATUSES = {"FAILED", "ERROR", "TIMEOUT", "ABORTED"}
_FINISHED_STATUSES = {"SUCCEEDED", "CANCELED", "PREEMPTED"}

# NANO feedback patterns for drawer states
_DRV_DONE_CLOSE_PAT = re.compile(r"^DRV DONE CLOSE (\d+)$", re.IGNORECASE)
_DRV_DONE_OPEN_PAT  = re.compile(r"^DRV DONE OPEN (\d+)$", re.IGNORECASE)
_DRV_ALREADY_HOME_PAT = re.compile(r"^OK D(\d+)_ALREADY_HOME$", re.IGNORECASE)
_DRV_ALREADY_OPEN_PAT = re.compile(r"^OK D(\d+)_ALREADY_OPEN$", re.IGNORECASE)
_DRV_ERROR_JAM_PAT = re.compile(r"^DRV ERROR JAM (\d+)$", re.IGNORECASE)

_MAX_JAM_RETRIES = 3


class StateTranslator:
    """
    Translates high-level robot state events into Nano serial commands.

    Instantiate once, inject a NanoInterface, then call the on_* handlers
    from your ROS2 subscribers / MQTT callbacks.
    """

    def __init__(self, nano: NanoInterface, on_state_update_cb=None):
        self._nano = nano
        self._on_state_update_cb = on_state_update_cb
        self._last_mode:   Optional[str] = None
        self._drawer_command_info: dict[int, dict[str, object]] = {}


    # ── service_feedback handler (ROS2 or MQTT — same JSON shape) ────────────
    def on_service_feedback(self, payload: dict) -> None:
        """
        payload example:
            {"command_id": "PATROL_21", "status": "EXECUTING"}
        """
        command_id = payload.get("command_id", "")
        status     = payload.get("status", "")

        # Error state overrides mode color
        if status.upper() in _ERROR_STATUSES:
            logger.info("Error status '%s' → LED OFF / error signal", status)
            # No dedicated ERROR mode in current firmware; use LED OFF as safe state.
            # Extend Nano firmware with LED MODE ERROR if Red is needed.
            self._nano.led_off()
            self._last_mode = None
            return

        if status.upper() in _FINISHED_STATUSES:
            logger.info("Status '%s' → LED Patrol mode (currently free)", status)
            self._nano.set_led_mode(LED_MODE_PATROL)
            self._last_mode = LED_MODE_PATROL
            return

        mode = self._resolve_mode(command_id)
        if mode and mode != self._last_mode:
            self._nano.set_led_mode(mode)
            self._last_mode = mode
            logger.debug("Mode changed → %s (command_id=%s)", mode, command_id)

    # ── /cmd_vel handler (LED motion disabled) ──────────────────────────────────────
    def on_cmd_vel(self, linear_x: float, angular_z: float) -> None:
        """Process velocity commands without emitting LED motion signals."""
        motion = self._classify_motion(linear_x, angular_z)
        logger.debug("Received cmd_vel (vx=%.3f ωz=%.3f) – LED motion disabled.", linear_x, angular_z)

    # ── Manual LED control handler (MQTT) ────────────────────────────────────
    def on_led_cmd(self, payload: dict) -> None:
        """
        payload example:
            {"cmd": "OFF"}
            or
            {"cmd": "MODE", "mode": "PATROL"}
        """
        cmd = payload.get("cmd", "").upper()
        if cmd == "OFF":
            self._nano.led_off()
            self._last_mode = None
            logger.info("Manual LED command: OFF")
        elif cmd == "MODE":
            mode = payload.get("mode", "").upper()
            try:
                self._nano.set_led_mode(mode)
                self._last_mode = mode
                logger.info("Manual LED command: MODE -> %s", mode)
            except ValueError as exc:
                logger.warning("Failed to set manual LED mode: %s", exc)
        else:
            logger.warning("Unknown manual LED command '%s'; ignoring.", cmd)

    # ── /robot/drawer/cmd handler (MQTT) ─────────────────────────────────────
    def on_drawer_cmd(self, payload: dict) -> None:
        """
        payload example:
            {"drawer": 1, "cmd": "OPEN"}
        """
        drawer_id = int(payload.get("drawer", 1))
        cmd       = payload.get("cmd", "").upper()

        if cmd == "OPEN":
            self._nano.open_drawer(drawer_id)
            self._drawer_command_info[drawer_id] = {
                "desired_state": "OPENED",
                "retry_count": 0,
            }
        elif cmd == "CLOSE":
            self._nano.close_drawer(drawer_id)
            self._drawer_command_info[drawer_id] = {
                "desired_state": "CLOSED",
                "retry_count": 0,
            }
        elif cmd == "HOME":
            self._nano.home_drawer()
            self._drawer_command_info.clear()
        elif cmd == "STOP":
            self._nano.stop_drawer()
            self._drawer_command_info.clear()
        else:
            logger.warning("Unknown drawer cmd '%s'; ignoring.", cmd)

    # ── Nano feedback handler ─────────────────────────────────────────────────
    def on_nano_feedback(self, line: str) -> None:
        """
        Called by NanoInterface._feedback_cb when the Nano sends a line back.
        Extend this with ROS2 topic publishing if needed.
        """
        parts = line.split()
        if not parts:
            return

        # Check drawer completion or status
        m_done_close = _DRV_DONE_CLOSE_PAT.match(line)
        m_done_open  = _DRV_DONE_OPEN_PAT.match(line)
        m_already_home = _DRV_ALREADY_HOME_PAT.match(line)
        m_already_open = _DRV_ALREADY_OPEN_PAT.match(line)
        m_jam = _DRV_ERROR_JAM_PAT.match(line)

        drawer_id = None
        state = None

        if m_jam:
            drawer_id = int(m_jam.group(1))

        if m_done_close:
            drawer_id = int(m_done_close.group(1))
            state = "CLOSED"
        elif m_done_open:
            drawer_id = int(m_done_open.group(1))
            state = "OPENED"
        elif m_already_home:
            drawer_id = int(m_already_home.group(1))
            state = "CLOSED"
        elif m_already_open:
            drawer_id = int(m_already_open.group(1))
            state = "OPENED"

        if drawer_id is not None and state is not None:
            logger.info("Drawer %d state update -> %s", drawer_id, state)
            
            # Check if this state update triggers the next step of a jam retry
            info = self._drawer_command_info.get(drawer_id)
            if info and info.get("retry_in_progress"):
                desired_state = info.get("desired_state")
                # If back-off succeeded (we reached the opposite state), send the retry command
                if (desired_state == "OPENED" and state == "CLOSED") or (desired_state == "CLOSED" and state == "OPENED"):
                    info["retry_in_progress"] = False
                    if desired_state == "OPENED":
                        logger.info("Drawer %d back-off complete, retrying OPEN", drawer_id)
                        self._nano.open_drawer(drawer_id)
                    else:
                        logger.info("Drawer %d back-off complete, retrying CLOSE", drawer_id)
                        self._nano.close_drawer(drawer_id)
                    state = None  # Intercept the intermediate state

            if state is not None:
                self._publish_drawer_state(drawer_id, state)
                self._clear_drawer_command_info(drawer_id, state)

        if m_jam:
            self._handle_drawer_jam(drawer_id)
            return

        if line.startswith("DRV DONE"):
            logger.info("Drawer done: %s", line)
        elif line.startswith("DRV ERROR"):
            logger.warning("Drawer error: %s", line)
            # Optionally trigger LED error state
            self._nano.led_off()
        elif line == "DRV HOMED":
            logger.info("Drawer homed successfully")
        elif line == "SYS READY":
            logger.info("Nano reported READY")
        elif line == "SYS PONG":
            logger.debug("Nano PONG received")
        elif line == "LED ACK OFF":
            logger.debug("Nano confirmed LED OFF")
        else:
            logger.debug("Unhandled Nano feedback: %s", line)

    # ── Private helpers ───────────────────────────────────────────────────────
    def _resolve_mode(self, command_id: str) -> Optional[str]:
        for pattern, mode in _MODE_PATTERNS:
            if pattern.match(command_id):
                return mode
        # Any other non-empty ID → GUIDANCE
        if command_id.strip():
            return LED_MODE_GUIDANCE
        return None

    def _publish_drawer_state(self, drawer_id: int, state: str) -> None:
        if self._on_state_update_cb:
            try:
                self._on_state_update_cb("robot/drawer/state", {"drawer": drawer_id, "state": state})
            except Exception as exc:
                logger.exception("Failed to publish drawer state MQTT message: %s", exc)

    def _clear_drawer_command_info(self, drawer_id: int, state: str) -> None:
        info = self._drawer_command_info.get(drawer_id)
        if info is None:
            return
        if info.get("desired_state") == state:
            self._drawer_command_info.pop(drawer_id, None)

    def _handle_drawer_jam(self, drawer_id: int) -> None:
        info = self._drawer_command_info.get(drawer_id)
        if info is None:
            logger.warning("Jam reported for drawer %d with no active command", drawer_id)
            self._publish_drawer_state(drawer_id, "JAMMED")
            self._nano.led_off()
            return

        retry_count = info.get("retry_count", 0)
        if retry_count >= _MAX_JAM_RETRIES:
            logger.error(
                "Drawer %d jammed after %d retries — intervention required",
                drawer_id,
                retry_count,
            )
            self._publish_drawer_state(drawer_id, "JAMMED")
            self._nano.stop_drawer()
            self._drawer_command_info.pop(drawer_id, None)
            return

        logger.warning(
            "Drawer %d jam detected during %s; retry %d/%d",
            drawer_id,
            info.get("desired_state"),
            retry_count + 1,
            _MAX_JAM_RETRIES,
        )
        self._publish_drawer_state(drawer_id, "JAM-RETRYING")
        self._nano.led_off()
        self._drawer_command_info[drawer_id]["retry_count"] = retry_count + 1

        desired_state = info.get("desired_state")
        if info.get("retry_in_progress"):
            info["retry_in_progress"] = False
            if desired_state == "OPENED":
                self._nano.open_drawer(drawer_id)
            else:
                self._nano.close_drawer(drawer_id)
        else:
            info["retry_in_progress"] = True
            if desired_state == "OPENED":
                self._nano.close_drawer(drawer_id)
            else:
                self._nano.open_drawer(drawer_id)

    @staticmethod
    def _classify_motion(linear_x: float, angular_z: float) -> str:
        """Classify motion for logging purposes only. No LED commands are emitted."""
        if linear_x > LINEAR_MOVE_THRESHOLD:
            return "FORWARD"
        if linear_x < -LINEAR_MOVE_THRESHOLD:
            return "REVERSE"
        if angular_z > ANGULAR_TURN_THRESHOLD:
            return "LEFT"
        if angular_z < -ANGULAR_TURN_THRESHOLD:
            return "RIGHT"
        return "STOP"
