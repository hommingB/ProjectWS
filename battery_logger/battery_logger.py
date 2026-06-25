#!/usr/bin/env python3
"""
Battery Telemetry Logger for Raspberry Pi 5
Subscribes to MQTT/Mosquitto and records battery SoC, current, and voltage
to SQLite (default, robust against power cuts) or CSV.
"""

import argparse
import csv
import json
import os
import sqlite3
import sys
import time
from datetime import datetime
import paho.mqtt.client as mqtt

# Default Configuration
DEFAULT_BROKER = "localhost"
DEFAULT_PORT = 1883
DEFAULT_TOPICS = ["robot/battery/status", "robot/electrical/battery"]
DEFAULT_DB_PATH = "battery_telemetry.db"
DEFAULT_CSV_PATH = "battery_telemetry.csv"


class BatteryLogger:
    def __init__(self, broker, port, topics, db_path, csv_path, use_csv=False, username=None, password=None):
        self.broker = broker
        self.port = port
        self.topics = topics
        self.db_path = os.path.abspath(db_path)
        self.csv_path = os.path.abspath(csv_path)
        self.use_csv = use_csv
        self.username = username
        self.password = password

        # Create base directory if it doesn't exist
        os.makedirs(os.path.dirname(self.db_path), exist_ok=True)
        os.makedirs(os.path.dirname(self.csv_path), exist_ok=True)

        if not self.use_csv:
            self._init_sqlite()
        else:
            self._init_csv()

    def _init_sqlite(self):
        """Initialize SQLite database and table structure."""
        print(f"[*] Initializing SQLite database at: {self.db_path}")
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        cursor.execute("""
            CREATE TABLE IF NOT EXISTS battery_telemetry (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                timestamp TEXT NOT NULL,
                soc REAL,
                voltage REAL,
                current REAL,
                power REAL,
                charging INTEGER,
                raw_payload TEXT
            )
        """)
        # Create an index on timestamp for fast queries
        cursor.execute("CREATE INDEX IF NOT EXISTS idx_timestamp ON battery_telemetry (timestamp)")
        conn.commit()
        conn.close()

    def _init_csv(self):
        """Initialize CSV file header if it doesn't exist."""
        print(f"[*] Initializing CSV logging at: {self.csv_path}")
        if not os.path.exists(self.csv_path):
            with open(self.csv_path, mode='w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(["timestamp", "soc", "voltage", "current", "power", "charging", "raw_payload"])

    def _parse_payload(self, payload_str):
        """
        Parses MQTT battery JSON payload.
        Handles different schemas:
        1. Flat schema: {"soc": 85.0, "voltage": 12.6, "current": -1.5, "charging": 0}
        2. Nested schema: {"data": {"soc_pct": 15.0, "voltage": 12.6, ...}}
        """
        try:
            data = json.loads(payload_str)
        except json.JSONDecodeError:
            print(f"[!] Failed to parse payload as JSON: {payload_str}")
            return None

        # Check for nested "data" dictionary (common in ROS/MQTT bridges)
        if "data" in data and isinstance(data["data"], dict):
            inner = data["data"]
        else:
            inner = data

        # Extract values with fallback mappings
        soc = inner.get("soc")
        if soc is None:
            soc = inner.get("soc_pct")
        if soc is None:
            soc = inner.get("battery_level")

        voltage = inner.get("voltage")
        if voltage is None:
            voltage = inner.get("v")

        current = inner.get("current")
        if current is None:
            current = inner.get("i")
            
        power = inner.get("power")
        if power is None and voltage is not None and current is not None:
            try:
                power = float(voltage) * float(current)
            except (ValueError, TypeError):
                power = None

        charging = inner.get("charging")
        if charging is None:
            charging_state = inner.get("charging_state")
            if charging_state is not None:
                charging = 1 if charging_state in [True, "charging", "CHARGING", 1, "1"] else 0

        # Convert types safely
        try:
            soc = float(soc) if soc is not None else None
            voltage = float(voltage) if voltage is not None else None
            current = float(current) if current is not None else None
            power = float(power) if power is not None else None
            charging = int(charging) if charging is not None else None
        except (ValueError, TypeError) as e:
            print(f"[!] Error casting payload values: {e}")

        return {
            "soc": soc,
            "voltage": voltage,
            "current": current,
            "power": power,
            "charging": charging,
            "raw_payload": payload_str
        }

    def log_telemetry(self, telemetry):
        """Store telemetry row to chosen storage backend."""
        timestamp = datetime.utcnow().isoformat()
        
        if not self.use_csv:
            try:
                conn = sqlite3.connect(self.db_path)
                cursor = conn.cursor()
                cursor.execute("""
                    INSERT INTO battery_telemetry (timestamp, soc, voltage, current, power, charging, raw_payload)
                    VALUES (?, ?, ?, ?, ?, ?, ?)
                """, (
                    timestamp,
                    telemetry["soc"],
                    telemetry["voltage"],
                    telemetry["current"],
                    telemetry["power"],
                    telemetry["charging"],
                    telemetry["raw_payload"]
                ))
                conn.commit()
                conn.close()
                print(f"[{timestamp}] SQLite Logged: SoC={telemetry['soc']}% V={telemetry['voltage']}V I={telemetry['current']}A")
            except sqlite3.Error as e:
                print(f"[!] SQLite Insert Error: {e}")
        else:
            try:
                with open(self.csv_path, mode='a', newline='') as f:
                    writer = csv.writer(f)
                    writer.writerow([
                        timestamp,
                        telemetry["soc"],
                        telemetry["voltage"],
                        telemetry["current"],
                        telemetry["power"],
                        telemetry["charging"],
                        telemetry["raw_payload"]
                    ])
                print(f"[{timestamp}] CSV Logged: SoC={telemetry['soc']}% V={telemetry['voltage']}V I={telemetry['current']}A")
            except Exception as e:
                print(f"[!] CSV Write Error: {e}")

    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            print(f"[*] Connected to MQTT Broker ({self.broker}:{self.port}) successfully.")
            for topic in self.topics:
                client.subscribe(topic)
                print(f"[*] Subscribed to topic: {topic}")
        else:
            print(f"[!] Connection failed with code {rc}")

    def on_message(self, client, userdata, msg):
        payload_str = msg.payload.decode("utf-8", errors="ignore")
        print(f"[*] Message received on {msg.topic}")
        telemetry = self._parse_payload(payload_str)
        if telemetry:
            self.log_telemetry(telemetry)

    def run(self):
        """Start the MQTT client loop."""
        try:
            # paho-mqtt >= 2.0.0
            client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION1)
        except AttributeError:
            # paho-mqtt < 2.0.0
            client = mqtt.Client()

        client.on_connect = self.on_connect
        client.on_message = self.on_message

        if self.username and self.password:
            client.username_pw_set(self.username, self.password)

        print(f"[*] Connecting to broker {self.broker}:{self.port}...")
        try:
            client.connect(self.broker, self.port, keepalive=60)
        except Exception as e:
            print(f"[!] Failed to connect to MQTT broker: {e}")
            sys.exit(1)

        # Blocking loop, handles reconnects automatically
        try:
            client.loop_forever()
        except KeyboardInterrupt:
            print("\n[*] Exiting battery logger gracefully.")
            sys.exit(0)


def export_sqlite_to_csv(db_path, csv_path):
    """Utility to export data from SQLite to a CSV file."""
    db_path = os.path.abspath(db_path)
    csv_path = os.path.abspath(csv_path)
    
    if not os.path.exists(db_path):
        print(f"[!] SQLite DB file not found: {db_path}")
        sys.exit(1)

    print(f"[*] Exporting SQLite ({db_path}) to CSV ({csv_path})...")
    try:
        conn = sqlite3.connect(db_path)
        cursor = conn.cursor()
        cursor.execute("SELECT timestamp, soc, voltage, current, power, charging, raw_payload FROM battery_telemetry ORDER BY timestamp ASC")
        rows = cursor.fetchall()
        
        with open(csv_path, mode='w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(["timestamp", "soc", "voltage", "current", "power", "charging", "raw_payload"])
            writer.writerows(rows)
            
        print(f"[+] Successfully exported {len(rows)} rows to {csv_path}")
        conn.close()
    except Exception as e:
        print(f"[!] Export failed: {e}")
        sys.exit(1)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Battery Telemetry MQTT Logger")
    parser.add_argument("--broker", default=DEFAULT_BROKER, help=f"MQTT broker hostname (default: {DEFAULT_BROKER})")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT, help=f"MQTT broker port (default: {DEFAULT_PORT})")
    parser.add_argument("--topics", nargs="+", default=DEFAULT_TOPICS, help=f"MQTT topics to subscribe to (default: {' '.join(DEFAULT_TOPICS)})")
    parser.add_argument("--db-path", default=DEFAULT_DB_PATH, help=f"SQLite database file path (default: {DEFAULT_DB_PATH})")
    parser.add_argument("--csv-path", default=DEFAULT_CSV_PATH, help=f"CSV file path (default: {DEFAULT_CSV_PATH})")
    parser.add_argument("--use-csv", action="store_true", help="Log directly to CSV instead of SQLite")
    parser.add_argument("--username", help="MQTT broker username")
    parser.add_argument("--password", help="MQTT broker password")
    parser.add_argument("--export", help="Export SQLite DB to specified CSV file path and exit")

    args = parser.parse_args()

    if args.export:
        export_sqlite_to_csv(args.db_path, args.export)
        sys.exit(0)

    logger = BatteryLogger(
        broker=args.broker,
        port=args.port,
        topics=args.topics,
        db_path=args.db_path,
        csv_path=args.csv_path,
        use_csv=args.use_csv,
        username=args.username,
        password=args.password
    )
    logger.run()
