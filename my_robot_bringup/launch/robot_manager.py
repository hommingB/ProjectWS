#!/usr/bin/env python3
"""
robot_manager.py
================
Runs on Pi 5 at boot via systemd. Responsibilities:
  1. Launch the full robot stack in dependency order
  2. Publish MQTT heartbeat so ESP32 knows Pi is alive
  3. Publish "ready" only after ALL stages have started successfully
  4. Monitor processes and restart non-critical ones on crash
  5. On shutdown request (ESP32 MQTT or SIGTERM):
       a. Deactivate Nav2 lifecycle nodes gracefully
       b. Terminate all processes in reverse launch order
       c. Publish "shutting_down" (ESP32 starts POST_ACK_DELAY countdown)
       d. Publish "offline" explicitly (broker is on Pi, LWT won't fire)
       e. OS shutdown

Startup order
─────────────
  Stage 1 — localization.launch.py  (RSP, diff_drive, sensors, ekf)
             waits for /odometry/filtered
  Stage 2 — navigation.launch.py    (twist_mux + Nav2 stack)
             waits for /navigate_to_pose action server
  Stage 3 — mqtt_bridge.launch.py
  Stage 4 — robot_commander.launch.py
  Stage 5 — robot_nano_bridge_node

MQTT status values published on TOPIC_PI_STATUS
────────────────────────────────────────────────
  "starting"      — Pi booted, launching stack
  "online"        — heartbeat (every HEARTBEAT_INTERVAL s)
  "ready"         — all stages up, robot operational
  "shutting_down" — shutdown sequence started
  "offline"       — published explicitly before MQTT disconnect

QoS notes
─────────
  Heartbeat / "online"   → QoS 0  (high-frequency, loss is fine)
  Status transitions      → QoS 1  (important, duplicates harmless)
  Commands                → QoS 1
  Retained telemetry      → QoS 1 + retain=True
  LWT (offline)           → QoS 1  (set at connect time)

Note on LWT: because the broker runs ON the Pi, LWT is only useful for
detecting unclean disconnects from external clients. For Pi shutdown,
"offline" is published explicitly in the shutdown sequence while Mosquitto
is still alive. The ESP32's PI_OFFLINE_TIMEOUT_MS timer also catches any
case where neither path works (full Pi crash).
"""

import os
import sys
import signal
import subprocess
import threading
import time
import logging
import json

import paho.mqtt.client as mqtt

# ─────────────────────────────────────────────────────────────────────────────
#  CONFIG — edit these for your setup
# ─────────────────────────────────────────────────────────────────────────────

MQTT_BROKER     = "localhost"
MQTT_PORT       = 1883
MQTT_CLIENT_ID  = "pi_manager"

# Topics — must match ESP32 config.h
TOPIC_PI_STATUS = "robot/pi/status"         # we publish here (QoS 1)
TOPIC_PS_EVENT  = "robot/powerswitch/event" # we subscribe
TOPIC_BATTERY   = "robot/battery/status"    # we subscribe (optional)

# ROS2 environment
ROS_SETUP       = "/opt/ros/jazzy/setup.bash"
WS_SETUP        = os.path.expanduser("~/ros2_project/install/setup.bash")

# Launch stages — executed in order, each waits for readiness before next
LAUNCH_STAGES = [
    {
        "name":          "localization",
        "cmd":           "ros2 launch my_robot_bringup localization.launch.py",
        "ready_topic":   "/odometry/filtered",  # wait for this topic
        "ready_timeout": 30,    # seconds before giving up
        "critical":      True,  # abort everything if this fails
    },
    {
        "name":          "navigation",
        "cmd":           "ros2 launch my_robot_bringup navigation.launch.py",
        "ready_topic":   "/navigate_to_pose/_action/status",
        "ready_timeout": 45,
        "critical":      True,
    },
    {
        "name":          "mqtt_bridge",
        "cmd":           "ros2 launch ros2_mqtt_bridge mqtt_bridge.launch.py",
        "ready_topic":   None,
        "ready_delay":   3,     # fixed delay when no topic to wait for
        "critical":      False,
    },
    {
        "name":          "robot_commander",
        "cmd":           "ros2 launch robot_commander_two robot_commander.launch.py",
        "ready_topic":   None,
        "ready_delay":   2,
        "critical":      False,
    },
    {
        "name":          "nano_bridge",
        "cmd":           "ros2 run robot_nano_bridge robot_nano_bridge_node",
        "ready_topic":   None,
        "ready_delay":   2,
        "critical":      False,
    },
]

# Nav2 lifecycle manager — service name for graceful shutdown
NAV2_LC_SERVICE         = "/lifecycle_manager_navigation/manage_nodes"
NAV2_SHUTDOWN_TIMEOUT   = 10   # s — wait for Nav2 to deactivate
PROCESS_KILL_TIMEOUT    = 5    # s — SIGTERM → SIGKILL grace period

# Heartbeat published while stack is running (must be << PI_OFFLINE_TIMEOUT_MS)
HEARTBEAT_INTERVAL      = 8    # s

# Crash recovery for non-critical processes
MAX_RESTARTS            = 3
RESTART_DELAY           = 5    # s between restart attempts

# ─────────────────────────────────────────────────────────────────────────────
#  Logging
# ─────────────────────────────────────────────────────────────────────────────

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    handlers=[
        logging.StreamHandler(sys.stdout),
        logging.FileHandler("/tmp/robot_manager.log"),
    ]
)
log = logging.getLogger("robot_manager")

# ─────────────────────────────────────────────────────────────────────────────
#  Global state
# ─────────────────────────────────────────────────────────────────────────────

_shutdown_requested = threading.Event()
_stack_ready        = False                        # set after all stages up
_processes: dict[str, subprocess.Popen] = {}
_restart_counts: dict[str, int] = {}
_mqtt_client: mqtt.Client | None = None
_ros_env: dict | None = None

# ─────────────────────────────────────────────────────────────────────────────
#  ROS2 environment
# ─────────────────────────────────────────────────────────────────────────────

def build_ros_env() -> dict:
    """Source ROS2 + workspace setup files, return the resulting env dict."""
    log.info("Sourcing ROS2 environment...")
    sources = [ROS_SETUP]
    if os.path.exists(WS_SETUP):
        sources.append(WS_SETUP)
    else:
        log.warning(f"Workspace setup not found: {WS_SETUP}")

    source_cmd = " && ".join(f"source {s}" for s in sources)
    result = subprocess.run(
        ["bash", "-c", f"{source_cmd} && env"],
        capture_output=True, text=True
    )
    if result.returncode != 0:
        log.error("Failed to source ROS2 environment:\n" + result.stderr)
        sys.exit(1)

    env = {}
    for line in result.stdout.splitlines():
        if "=" in line:
            k, _, v = line.partition("=")
            env[k] = v
    log.info(f"ROS2 env ready (distro={env.get('ROS_DISTRO', '?')})")
    return env

# ─────────────────────────────────────────────────────────────────────────────
#  Process management
# ─────────────────────────────────────────────────────────────────────────────

def launch_process(name: str, cmd: str) -> subprocess.Popen:
    log.info(f"[{name}] Launching: {cmd}")
    proc = subprocess.Popen(
        cmd, shell=True, env=_ros_env,
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        text=True,
        preexec_fn=os.setsid,   # own process group → can kill whole tree
    )
    threading.Thread(target=_stream_log, args=(name, proc), daemon=True).start()
    return proc


def _stream_log(name: str, proc: subprocess.Popen):
    try:
        for line in proc.stdout:
            log.debug(f"[{name}] {line.rstrip()}")
    except Exception:
        pass


def stop_process(name: str, proc: subprocess.Popen):
    if proc.poll() is not None:
        return
    log.info(f"[{name}] SIGTERM...")
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
    except ProcessLookupError:
        return
    try:
        proc.wait(timeout=PROCESS_KILL_TIMEOUT)
        log.info(f"[{name}] Exited cleanly")
    except subprocess.TimeoutExpired:
        log.warning(f"[{name}] Timeout — SIGKILL")
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except ProcessLookupError:
            pass


def wait_for_topic(topic: str, timeout: int) -> bool:
    """Poll `ros2 topic list` until topic appears or timeout elapses."""
    log.info(f"Waiting for {topic} (timeout={timeout}s)...")
    deadline = time.time() + timeout
    while time.time() < deadline:
        if _shutdown_requested.is_set():
            return False
        r = subprocess.run(
            "ros2 topic list", shell=True, env=_ros_env,
            capture_output=True, text=True
        )
        if topic in r.stdout:
            log.info(f"Topic {topic} up")
            return True
        time.sleep(2)
    log.warning(f"Timeout waiting for {topic}")
    return False

# ─────────────────────────────────────────────────────────────────────────────
#  Nav2 graceful shutdown
# ─────────────────────────────────────────────────────────────────────────────

def shutdown_nav2_gracefully():
    """
    Call lifecycle_manager's manage_nodes service with SHUTDOWN (4).
    This deactivates → cleans up all Nav2 lifecycle nodes in correct order.
    Commands: 0=STARTUP, 1=PAUSE, 2=RESUME, 3=RESET, 4=SHUTDOWN
    """
    log.info("Nav2 lifecycle SHUTDOWN...")
    cmd = (
        f"ros2 service call {NAV2_LC_SERVICE} "
        "nav2_msgs/srv/ManageLifecycleNodes '{command: 4}'"
    )
    try:
        result = subprocess.run(
            cmd, shell=True, env=_ros_env,
            timeout=NAV2_SHUTDOWN_TIMEOUT,
            capture_output=True, text=True
        )
        if result.returncode == 0:
            log.info("Nav2 shutdown acknowledged")
        else:
            log.warning(f"Nav2 shutdown returned error: {result.stderr.strip()}")
    except subprocess.TimeoutExpired:
        log.warning("Nav2 shutdown timed out — continuing")

# ─────────────────────────────────────────────────────────────────────────────
#  MQTT
# ─────────────────────────────────────────────────────────────────────────────

def mqtt_on_connect(client, userdata, flags, reason_code, properties):
    if reason_code == 0:
        log.info("MQTT connected")
        # Subscribe with QoS 1 — important messages, duplicates harmless
        client.subscribe(TOPIC_PS_EVENT, qos=1)
        client.subscribe(TOPIC_BATTERY,  qos=0)  # telemetry, QoS 0 fine
        # Immediately announce we're starting up
        client.publish(TOPIC_PI_STATUS, "starting", qos=1, retain=False)
    else:
        log.warning(f"MQTT connect failed rc={reason_code}")


def mqtt_on_message(client, userdata, msg):
    topic   = msg.topic
    payload = msg.payload.decode(errors="replace").strip()

    if topic == TOPIC_PS_EVENT:
        try:
            data = json.loads(payload)
        except json.JSONDecodeError:
            return
        event = data.get("event", "")

        if event == "shutdown_request":
            log.info(f"[MQTT] Shutdown request from ESP32 (SoC={data.get('soc','?')}%)")
            _shutdown_requested.set()
        elif event == "soc_low":
            log.warning(f"[MQTT] Battery low — SoC={data.get('soc','?')}%")
        elif event == "soc_critical":
            log.error(f"[MQTT] Battery CRITICAL — SoC={data.get('soc','?')}%")
        elif event == "power_off":
            log.info("[MQTT] ESP32 power cut confirmed")


def publish_status(status: str, qos: int = 1):
    """
    Publish Pi status on TOPIC_PI_STATUS.
    QoS 1 for state transitions, QoS 0 for frequent heartbeats.
    retain=False — consumers should always see the latest live value,
    not a stale retained one from a previous session.
    """
    if _mqtt_client and _mqtt_client.is_connected():
        _mqtt_client.publish(TOPIC_PI_STATUS, status, qos=qos, retain=False)
        log.info(f"[MQTT] status → '{status}'")


def heartbeat_thread():
    """
    Publish heartbeat every HEARTBEAT_INTERVAL seconds.
      - "online"  while stack is starting / running normally
      - "ready"   once all stages are up (_stack_ready flag)
    QoS 0 — high frequency, occasional loss acceptable.
    """
    while not _shutdown_requested.is_set():
        status = "ready" if _stack_ready else "online"
        publish_status(status, qos=0)
        _shutdown_requested.wait(timeout=HEARTBEAT_INTERVAL)


def setup_mqtt() -> mqtt.Client:
    client = mqtt.Client(
        mqtt.CallbackAPIVersion.VERSION2,
        client_id=MQTT_CLIENT_ID
    )
    client.on_connect = mqtt_on_connect
    client.on_message = mqtt_on_message

    # LWT — broker delivers this if TCP drops without a DISCONNECT packet.
    # Note: on clean OS shutdown the broker dies too, so LWT may not reach
    # subscribers. "offline" is therefore also published explicitly in shutdown().
    # QoS 1 so broker retries delivery if a subscriber reconnects.
    client.will_set(TOPIC_PI_STATUS, "offline", qos=1, retain=False)

    log.info(f"Connecting to MQTT {MQTT_BROKER}:{MQTT_PORT}...")
    for attempt in range(10):
        try:
            client.connect(MQTT_BROKER, MQTT_PORT, keepalive=30)
            break
        except Exception as e:
            log.warning(f"MQTT attempt {attempt+1}/10 failed: {e} — retry in 3s")
            time.sleep(3)
    else:
        log.error("MQTT broker unreachable — running without MQTT")
        return client

    client.loop_start()
    return client

# ─────────────────────────────────────────────────────────────────────────────
#  Process monitor — restart crashed non-critical processes
# ─────────────────────────────────────────────────────────────────────────────

def monitor_thread():
    while not _shutdown_requested.is_set():
        for stage in LAUNCH_STAGES:
            name = stage["name"]
            if name not in _processes:
                continue
            proc = _processes[name]
            if proc.poll() is None:
                continue    # still running, all good

            if _shutdown_requested.is_set():
                break

            if stage.get("critical", False):
                log.error(f"[{name}] Critical process died — shutting down")
                _shutdown_requested.set()
                break

            count = _restart_counts.get(name, 0)
            if count >= MAX_RESTARTS:
                log.error(f"[{name}] Max restarts reached, giving up")
                continue

            log.warning(f"[{name}] Restarting ({count+1}/{MAX_RESTARTS})...")
            time.sleep(RESTART_DELAY)
            _processes[name]      = launch_process(name, stage["cmd"])
            _restart_counts[name] = count + 1

        _shutdown_requested.wait(timeout=5)

# ─────────────────────────────────────────────────────────────────────────────
#  Startup sequence
# ─────────────────────────────────────────────────────────────────────────────

def startup() -> bool:
    """
    Launch each stage in order, waiting for its readiness signal.
    Returns True if all critical stages came up successfully.
    Publishes "ready" (via _stack_ready flag) only after ALL stages complete.
    """
    global _stack_ready
    log.info("=== Robot stack starting ===")

    for stage in LAUNCH_STAGES:
        if _shutdown_requested.is_set():
            return False

        name     = stage["name"]
        critical = stage.get("critical", False)

        proc = launch_process(name, stage["cmd"])
        _processes[name]      = proc
        _restart_counts[name] = 0

        # Wait for readiness
        topic   = stage.get("ready_topic")
        delay   = stage.get("ready_delay", 0)
        timeout = stage.get("ready_timeout", 20)

        if topic:
            ready = wait_for_topic(topic, timeout)
            if not ready and critical:
                log.error(f"[{name}] Critical stage not ready — aborting startup")
                return False
        elif delay:
            log.info(f"[{name}] Settling ({delay}s)...")
            time.sleep(delay)

        if proc.poll() is not None:
            if critical:
                log.error(f"[{name}] Exited immediately — aborting startup")
                return False
            log.warning(f"[{name}] Exited immediately (non-critical)")

        log.info(f"[{name}] ✓ Stage up")

    # All stages complete — set the flag; heartbeat_thread picks it up
    _stack_ready = True
    # Publish "ready" immediately at QoS 1 (don't wait for next heartbeat tick)
    publish_status("ready", qos=1)
    log.info("=== All stages up — robot READY ===")
    return True

# ─────────────────────────────────────────────────────────────────────────────
#  Shutdown sequence
# ─────────────────────────────────────────────────────────────────────────────

def shutdown():
    """
    Ordered shutdown:
      1. Nav2 lifecycle SHUTDOWN (deactivates managed nodes cleanly)
      2. Stop all processes in reverse launch order
      3. Publish "shutting_down" — ESP32 starts POST_ACK_DELAY countdown
      4. Publish "offline" explicitly — broker is on Pi, LWT won't fire
      5. Disconnect MQTT cleanly
      6. OS shutdown — ESP32 cuts power after POST_ACK_DELAY_MS
    """
    log.info("=== Shutdown sequence starting ===")

    # 1. Nav2 graceful deactivation
    if "navigation" in _processes and _processes["navigation"].poll() is None:
        shutdown_nav2_gracefully()

    # 2. Stop processes in reverse order
    for stage in reversed(LAUNCH_STAGES):
        name = stage["name"]
        if name in _processes:
            stop_process(name, _processes[name])

    # 3 & 4. Tell ESP32 and any remaining subscribers what's happening.
    # Both publishes happen while Mosquitto is still running.
    # "shutting_down" triggers ESP32 POST_ACK_DELAY countdown.
    # "offline"       clears any subscriber's "Pi is online" state.
    publish_status("shutting_down", qos=1)
    time.sleep(0.5)
    publish_status("offline", qos=1)
    time.sleep(0.5)

    # 5. Disconnect cleanly (suppresses LWT since we sent it manually)
    if _mqtt_client:
        _mqtt_client.loop_stop()
        _mqtt_client.disconnect()

    log.info("=== ROS2 stack stopped, MQTT disconnected ===")

    # 6. OS shutdown — Linux halts within ~5-8 s, well inside POST_ACK_DELAY
    subprocess.run(["sudo", "shutdown", "-h", "now"])

# ─────────────────────────────────────────────────────────────────────────────
#  Signal handlers
# ─────────────────────────────────────────────────────────────────────────────

def handle_signal(signum, frame):
    log.info(f"Signal {signum} received — shutdown requested")
    _shutdown_requested.set()

# ─────────────────────────────────────────────────────────────────────────────
#  Entry point
# ─────────────────────────────────────────────────────────────────────────────

def main():
    global _ros_env, _mqtt_client

    signal.signal(signal.SIGTERM, handle_signal)
    signal.signal(signal.SIGINT,  handle_signal)

    _ros_env     = build_ros_env()
    _mqtt_client = setup_mqtt()

    # Heartbeat starts immediately — publishes "online" until stack is ready,
    # then switches to "ready" automatically via _stack_ready flag
    threading.Thread(target=heartbeat_thread, daemon=True).start()

    ok = startup()
    if not ok:
        log.error("Startup failed")
        _shutdown_requested.set()

    # Notify systemd the service is ready (requires `pip install sdnotify`)
    try:
        import sdnotify
        sdnotify.SystemdNotifier().notify("READY=1")
    except ImportError:
        pass

    # Start process monitor
    threading.Thread(target=monitor_thread, daemon=True).start()

    log.info("Waiting for shutdown signal...")
    _shutdown_requested.wait()

    shutdown()


if __name__ == "__main__":
    main()