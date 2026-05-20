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
    LED_MOTION_FORWARD, LED_MOTION_REVERSE,
    LED_MOTION_LEFT, LED_MOTION_RIGHT, LED_MOTION_STOP,
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


class StateTranslator:
    """
    Translates high-level robot state events into Nano serial commands.

    Instantiate once, inject a NanoInterface, then call the on_* handlers
    from your ROS2 subscribers / MQTT callbacks.
    """

    def __init__(self, nano: NanoInterface):
        self._nano = nano
        self._last_mode:   Optional[str] = None
        self._last_motion: Optional[str] = None

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

        mode = self._resolve_mode(command_id)
        if mode and mode != self._last_mode:
            self._nano.set_led_mode(mode)
            self._last_mode = mode
            logger.debug("Mode changed → %s (command_id=%s)", mode, command_id)

    # ── /cmd_vel handler ──────────────────────────────────────────────────────
    def on_cmd_vel(self, linear_x: float, angular_z: float) -> None:
        """
        Derive motion state from Nav2 velocity commands and update LEDs.
        Priority: linear movement takes precedence over turning in place.
        """
        motion = self._classify_motion(linear_x, angular_z)
        if motion != self._last_motion:
            self._nano.set_led_motion(motion)
            self._last_motion = motion
            logger.debug("Motion changed → %s (vx=%.3f ωz=%.3f)",
                         motion, linear_x, angular_z)

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
        elif cmd == "CLOSE":
            self._nano.close_drawer(drawer_id)
        elif cmd == "HOME":
            self._nano.home_drawer()
        elif cmd == "STOP":
            self._nano.stop_drawer()
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

    @staticmethod
    def _classify_motion(linear_x: float, angular_z: float) -> str:
        if linear_x > LINEAR_MOVE_THRESHOLD:
            return LED_MOTION_FORWARD
        if linear_x < -LINEAR_MOVE_THRESHOLD:
            return LED_MOTION_REVERSE
        if angular_z > ANGULAR_TURN_THRESHOLD:
            return LED_MOTION_LEFT
        if angular_z < -ANGULAR_TURN_THRESHOLD:
            return LED_MOTION_RIGHT
        return LED_MOTION_STOP
