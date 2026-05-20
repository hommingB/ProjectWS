"""
nano_interface.py
-----------------
Serial abstraction layer between Raspberry Pi (ROS2/MQTT) and Arduino Nano.
All protocol string construction is centralized here — never write raw serial
strings anywhere else in the codebase.

Fault tolerance
---------------
- Serial port is opened in a background reconnect loop; the node starts even
  if the Nano is not plugged in yet.
- If the port disconnects at runtime, the reconnect loop retries with
  exponential back-off (1 s → 2 s → 4 s … up to MAX_RETRY_DELAY).
- Commands issued while disconnected are silently dropped and logged as
  warnings — callers never raise.
- The reader thread survives transient SerialException errors and re-enters
  the reconnect loop automatically.
"""

import serial
import threading
import logging
import time
from typing import Callable, Optional

logger = logging.getLogger(__name__)

# ── Valid constants (used by callers for type-safety) ─────────────────────────
DRAWER_OPEN  = "OPEN"
DRAWER_CLOSE = "CLOSE"
DRAWER_HOME  = "HOME"
DRAWER_STOP  = "STOP"

LED_MODE_PATROL   = "PATROL"
LED_MODE_GUIDANCE = "GUIDANCE"
LED_MODE_DOCKING  = "DOCKING"

LED_MOTION_FORWARD = "FORWARD"
LED_MOTION_REVERSE = "REVERSE"
LED_MOTION_LEFT    = "LEFT"
LED_MOTION_RIGHT   = "RIGHT"
LED_MOTION_STOP    = "STOP"

# ── Retry tuning ──────────────────────────────────────────────────────────────
INITIAL_RETRY_DELAY = 1.0    # seconds before first reconnect attempt
MAX_RETRY_DELAY     = 30.0   # cap on back-off
BACKOFF_FACTOR      = 2.0    # multiply delay by this on each failure


class NanoInterface:
    """
    High-level Python abstraction over the text serial protocol spoken
    by the Arduino Nano peripheral controller.

    The connection is managed entirely in the background — callers just
    call the public methods and the class handles connect / reconnect
    transparently.

    Usage
    -----
    nano = NanoInterface(port="/dev/ttyUSB0", baud=115200)
    nano.connect()          # starts background thread, returns immediately
    nano.set_led_mode("PATROL")
    nano.open_drawer(1)
    nano.disconnect()

    Or as a context manager:
    with NanoInterface("/dev/ttyUSB0") as nano:
        nano.set_led_motion("FORWARD")
    """

    def __init__(
        self,
        port: str = "/dev/ttyUSB0",
        baud: int = 115200,
        timeout: float = 1.0,
        feedback_callback: Optional[Callable[[str], None]] = None,
    ):
        self._port        = port
        self._baud        = baud
        self._timeout     = timeout
        self._feedback_cb = feedback_callback

        self._serial: Optional[serial.Serial] = None
        self._lock        = threading.Lock()   # guards _serial writes
        self._running     = False
        self._connected   = False              # True only when port is open

        self._reconnect_thread: Optional[threading.Thread] = None
        self._reader_thread:    Optional[threading.Thread] = None

    # ── Public: connection lifecycle ──────────────────────────────────────────
    def connect(self) -> None:
        """Start background reconnect + reader loops. Returns immediately."""
        self._running = True
        self._reconnect_thread = threading.Thread(
            target=self._reconnect_loop, daemon=True, name="nano-reconnect"
        )
        self._reconnect_thread.start()
        logger.info("NanoInterface started (port=%s, baud=%d)", self._port, self._baud)

    def disconnect(self) -> None:
        """Signal all threads to stop and close the port cleanly."""
        logger.info("NanoInterface disconnecting…")
        self._running    = False
        self._connected  = False
        if self._reader_thread:
            self._reader_thread.join(timeout=2.0)
        if self._reconnect_thread:
            self._reconnect_thread.join(timeout=3.0)
        self._close_port()
        logger.info("NanoInterface disconnected")

    @property
    def is_connected(self) -> bool:
        return self._connected

    def __enter__(self) -> "NanoInterface":
        self.connect()
        return self

    def __exit__(self, *_) -> None:
        self.disconnect()

    # ── Public: drawer commands ───────────────────────────────────────────────
    def open_drawer(self, drawer_id: int) -> None:
        self._send(f"DRV OPEN {drawer_id}")

    def close_drawer(self, drawer_id: int) -> None:
        self._send(f"DRV CLOSE {drawer_id}")

    def home_drawer(self) -> None:
        self._send("DRV HOME")

    def stop_drawer(self) -> None:
        self._send("DRV STOP")

    # ── Public: LED commands ──────────────────────────────────────────────────
    def set_led_mode(self, mode: str) -> None:
        valid = {LED_MODE_PATROL, LED_MODE_GUIDANCE, LED_MODE_DOCKING}
        if mode not in valid:
            raise ValueError(f"Invalid LED mode '{mode}'. Valid: {valid}")
        self._send(f"LED MODE {mode}")

    def set_led_motion(self, motion: str) -> None:
        valid = {
            LED_MOTION_FORWARD, LED_MOTION_REVERSE,
            LED_MOTION_LEFT, LED_MOTION_RIGHT, LED_MOTION_STOP,
        }
        if motion not in valid:
            raise ValueError(f"Invalid LED motion '{motion}'. Valid: {valid}")
        self._send(f"LED MOTION {motion}")

    def led_off(self) -> None:
        self._send("LED OFF")

    # ── Public: system commands ───────────────────────────────────────────────
    def ping(self) -> None:
        self._send("SYS PING")

    def emergency_stop(self) -> None:
        """Highest-priority command — always logged, even when dropped."""
        if not self._connected:
            logger.error("ESTOP requested but Nano not connected — command dropped!")
        self._send("SYS ESTOP")

    # ── Private: reconnect loop ───────────────────────────────────────────────
    def _reconnect_loop(self) -> None:
        """
        Runs while self._running is True.
        Opens the port, starts the reader thread, then blocks until the reader
        exits (meaning the port died). Then retries with exponential back-off.
        """
        delay = INITIAL_RETRY_DELAY
        while self._running:
            try:
                logger.info("Attempting to open serial port %s …", self._port)
                ser = serial.Serial(
                    port=self._port,
                    baudrate=self._baud,
                    timeout=self._timeout,
                )
                with self._lock:
                    self._serial = ser
                self._connected = True
                delay = INITIAL_RETRY_DELAY   # reset back-off on success
                logger.info("Serial port %s opened successfully", self._port)

                # Start reader; it runs until the port breaks or _running=False
                self._reader_thread = threading.Thread(
                    target=self._read_loop, daemon=True, name="nano-reader"
                )
                self._reader_thread.start()
                self._reader_thread.join()    # block here until reader exits

                if self._running:
                    logger.warning(
                        "Serial reader exited unexpectedly on %s — will reconnect in %.0f s",
                        self._port, delay,
                    )

            except serial.SerialException as exc:
                logger.warning(
                    "Cannot open %s: %s — retrying in %.0f s",
                    self._port, exc, delay,
                )
            except Exception as exc:
                logger.error(
                    "Unexpected error opening serial port %s: %s — retrying in %.0f s",
                    self._port, exc, delay,
                )
            finally:
                self._connected = False
                self._close_port()

            # Interruptible sleep so disconnect() is responsive
            self._interruptible_sleep(delay)
            delay = min(delay * BACKOFF_FACTOR, MAX_RETRY_DELAY)

        logger.debug("Reconnect loop exiting")

    # ── Private: reader loop ──────────────────────────────────────────────────
    def _read_loop(self) -> None:
        """
        Reads lines from the Nano and dispatches to feedback_callback.
        Exits on SerialException so the reconnect loop can take over.
        """
        logger.debug("Serial reader thread started")
        while self._running and self._connected:
            try:
                ser = self._serial
                if ser is None or not ser.is_open:
                    break
                if ser.in_waiting:
                    raw  = ser.readline()
                    line = raw.decode("ascii", errors="replace").strip()
                    if line:
                        logger.debug("← Nano: %s", line)
                        if self._feedback_cb:
                            try:
                                self._feedback_cb(line)
                            except Exception:
                                logger.exception("Error in feedback callback")
                else:
                    time.sleep(0.005)

            except serial.SerialException as exc:
                logger.error("Serial read error on %s: %s — triggering reconnect", self._port, exc)
                self._connected = False
                break
            except Exception as exc:
                logger.exception("Unexpected error in serial reader: %s", exc)
                self._connected = False
                break

        logger.debug("Serial reader thread exiting")

    # ── Private: send ─────────────────────────────────────────────────────────
    def _send(self, command: str) -> None:
        """Write one command line. Drops silently if not connected."""
        if not self._connected or self._serial is None:
            logger.warning("Nano not connected — dropping: %s", command)
            return
        line = (command.strip() + "\n").encode("ascii")
        try:
            with self._lock:
                self._serial.write(line)
            logger.debug("→ Nano: %s", command.strip())
        except serial.SerialException as exc:
            logger.error("Write to Nano failed (%s) — triggering reconnect", exc)
            self._connected = False

    # ── Private: helpers ──────────────────────────────────────────────────────
    def _close_port(self) -> None:
        with self._lock:
            if self._serial:
                try:
                    self._serial.close()
                except Exception:
                    pass
                self._serial = None

    def _interruptible_sleep(self, duration: float) -> None:
        """Sleep in short slices so disconnect() wakes us quickly."""
        end = time.monotonic() + duration
        while self._running and time.monotonic() < end:
            time.sleep(0.2)