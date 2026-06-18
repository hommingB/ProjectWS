"""
test_bridge.py
--------------
Unit tests for StateTranslator (pure logic, no serial hardware needed).
Run with:  python -m pytest test_bridge.py -v
"""

import pytest
from unittest.mock import MagicMock, call, patch
from robot_nano_bridge.state_translator import StateTranslator, ANGULAR_TURN_THRESHOLD, LINEAR_MOVE_THRESHOLD, _MAX_JAM_RETRIES


@pytest.fixture
def nano():
    """Mock NanoInterface."""
    m = MagicMock()
    return m


@pytest.fixture
def translator(nano):
    return StateTranslator(nano)


# ── LED mode resolution ───────────────────────────────────────────────────────
class TestLedMode:
    def test_patrol_pattern(self, translator, nano):
        translator.on_service_feedback({"command_id": "PATROL_21", "status": "EXECUTING"})
        nano.set_led_mode.assert_called_once_with("PATROL")

    def test_dock_pattern(self, translator, nano):
        translator.on_service_feedback({"command_id": "DOCK_03", "status": "EXECUTING"})
        nano.set_led_mode.assert_called_once_with("DOCKING")

    def test_random_guid_is_guidance(self, translator, nano):
        translator.on_service_feedback({"command_id": "a1b2c3d4-5678", "status": "EXECUTING"})
        nano.set_led_mode.assert_called_once_with("GUIDANCE")

    def test_error_status_turns_led_off(self, translator, nano):
        translator.on_service_feedback({"command_id": "PATROL_5", "status": "FAILED"})
        nano.led_off.assert_called_once()
        nano.set_led_mode.assert_not_called()

    def test_finished_status_turns_led_to_patrol(self, translator, nano):
        translator.on_service_feedback({"command_id": "GUID_5", "status": "SUCCEEDED"})
        nano.set_led_mode.assert_called_once_with("PATROL")

    def test_canceled_status_turns_led_to_patrol(self, translator, nano):
        translator.on_service_feedback({"command_id": "GUID_5", "status": "CANCELED"})
        nano.set_led_mode.assert_called_once_with("PATROL")

    def test_preempted_status_turns_led_to_patrol(self, translator, nano):
        translator.on_service_feedback({"command_id": "GUID_5", "status": "PREEMPTED"})
        nano.set_led_mode.assert_called_once_with("PATROL")

    def test_mode_deduplication(self, translator, nano):
        """Same mode twice → only one command sent."""
        translator.on_service_feedback({"command_id": "PATROL_1", "status": "EXECUTING"})
        translator.on_service_feedback({"command_id": "PATROL_2", "status": "EXECUTING"})
        assert nano.set_led_mode.call_count == 1


# ── Manual LED commands ────────────────────────────────────────────────────────
class TestLedManualCmd:
    def test_manual_led_off(self, translator, nano):
        translator.on_led_cmd({"cmd": "OFF"})
        nano.led_off.assert_called_once()

    def test_manual_led_mode_valid(self, translator, nano):
        translator.on_led_cmd({"cmd": "MODE", "mode": "PATROL"})
        nano.set_led_mode.assert_called_once_with("PATROL")

    def test_manual_led_mode_invalid_ignored(self, translator, nano):
        # ValueError is caught inside state_translator and logged
        translator.on_led_cmd({"cmd": "MODE", "mode": "INVALID_COLOR"})
        # nano.set_led_mode will still be called (where it raises ValueError)
        nano.set_led_mode.assert_called_once_with("INVALID_COLOR")

    def test_manual_led_unknown_cmd_ignored(self, translator, nano):
        translator.on_led_cmd({"cmd": "DANCE"})
        nano.led_off.assert_not_called()
        nano.set_led_mode.assert_not_called()


# ── Drawer commands ───────────────────────────────────────────────────────────
class TestDrawer:
    def test_open_drawer(self, translator, nano):
        translator.on_drawer_cmd({"drawer": 1, "cmd": "OPEN"})
        nano.open_drawer.assert_called_once_with(1)

    def test_close_drawer(self, translator, nano):
        translator.on_drawer_cmd({"drawer": 2, "cmd": "CLOSE"})
        nano.close_drawer.assert_called_once_with(2)

    def test_home_drawer(self, translator, nano):
        translator.on_drawer_cmd({"drawer": 1, "cmd": "HOME"})
        nano.home_drawer.assert_called_once()

    def test_stop_drawer(self, translator, nano):
        translator.on_drawer_cmd({"drawer": 1, "cmd": "STOP"})
        nano.stop_drawer.assert_called_once()

    def test_unknown_cmd_ignored(self, translator, nano):
        translator.on_drawer_cmd({"drawer": 1, "cmd": "SPIN"})
        nano.open_drawer.assert_not_called()
        nano.close_drawer.assert_not_called()


# ── Nano feedback parsing ─────────────────────────────────────────────────────
class TestNanoFeedback:
    def test_drv_error_triggers_led_off(self, translator, nano):
        translator.on_nano_feedback("DRV ERROR JAM 1")
        nano.led_off.assert_called_once()

    def test_other_feedback_no_crash(self, translator, nano):
        translator.on_nano_feedback("SYS READY")
        translator.on_nano_feedback("SYS PONG")
        translator.on_nano_feedback("DRV DONE OPEN 1")
        translator.on_nano_feedback("DRV HOMED")
        translator.on_nano_feedback("LED ACK OFF")
        # none of these should raise


# ── Drawer state feedback (MQTT publish) ──────────────────────────────────────
class TestDrawerStateFeedback:
    def test_drv_done_close(self, nano):
        cb = MagicMock()
        translator = StateTranslator(nano, on_state_update_cb=cb)
        translator.on_nano_feedback("DRV DONE CLOSE 2")
        cb.assert_called_once_with("robot/drawer/state", {"drawer": 2, "state": "CLOSED"})

    def test_drv_done_open(self, nano):
        cb = MagicMock()
        translator = StateTranslator(nano, on_state_update_cb=cb)
        translator.on_nano_feedback("DRV DONE OPEN 3")
        cb.assert_called_once_with("robot/drawer/state", {"drawer": 3, "state": "OPENED"})

    def test_already_home(self, nano):
        cb = MagicMock()
        translator = StateTranslator(nano, on_state_update_cb=cb)
        translator.on_nano_feedback("OK D1_ALREADY_HOME")
        cb.assert_called_once_with("robot/drawer/state", {"drawer": 1, "state": "CLOSED"})

    def test_already_open(self, nano):
        cb = MagicMock()
        translator = StateTranslator(nano, on_state_update_cb=cb)
        translator.on_nano_feedback("OK D4_ALREADY_OPEN")
        cb.assert_called_once_with("robot/drawer/state", {"drawer": 4, "state": "OPENED"})

    def test_drv_error_jam_retries_open(self, nano):
        cb = MagicMock()
        translator = StateTranslator(nano, on_state_update_cb=cb)
        translator.on_drawer_cmd({"drawer": 1, "cmd": "OPEN"})
        translator.on_nano_feedback("DRV ERROR JAM 1")

        assert cb.call_args_list[-1] == call(
            "robot/drawer/state",
            {"drawer": 1, "state": "JAM-RETRYING"},
        )
        assert nano.mock_calls == [
            call.open_drawer(1),
            call.led_off(),
            call.close_drawer(1),
        ]

        # Feed back-off completion to trigger actual retry
        translator.on_nano_feedback("DRV DONE CLOSE 1")
        assert nano.mock_calls == [
            call.open_drawer(1),
            call.led_off(),
            call.close_drawer(1),
            call.open_drawer(1),
        ]

    def test_drv_error_jam_becomes_jammed_after_max_retries(self, nano):
        cb = MagicMock()
        translator = StateTranslator(nano, on_state_update_cb=cb)
        translator.on_drawer_cmd({"drawer": 2, "cmd": "CLOSE"})

        for _ in range(_MAX_JAM_RETRIES + 1):
            translator.on_nano_feedback("DRV ERROR JAM 2")

        assert cb.call_args_list[-1] == call(
            "robot/drawer/state",
            {"drawer": 2, "state": "JAMMED"},
        )
        nano.stop_drawer.assert_called_once()
