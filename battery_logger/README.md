# Battery Telemetry Logger & Plotter (Pi 5)

A lightweight python utility designed to subscribe to battery telemetry data via a local MQTT broker (Mosquitto) on a Raspberry Pi 5. It parses incoming telemetry and stores it securely in a SQLite database (recommended for power outage robustness) or directly to a CSV file. It also includes tools for exporting data and generating visualization plots.

## Project Structure

- `battery_logger.py`: The main daemon that subscribes to Mosquitto and records telemetry to SQLite or CSV.
- `plot_battery.py`: Visualizer script that reads SQLite or CSV data and generates a clean, multi-panel time-series chart (`battery_plot.png`).
- `battery_logger.service`: systemd service configuration file to run the logger continuously as a background daemon on boot.

---

## Installation & Setup

On Raspberry Pi 5, you can install the required packages globally using system package management (`apt`), which conforms to PEP 668 without virtual environments:

### 1. Install Dependencies Globally
Run the following command:

```bash
sudo apt update
sudo apt install -y python3-paho-mqtt python3-matplotlib
```

### 2. Supported JSON Formats
The logger dynamically handles multiple JSON schemas:
- **Flat Format** (Default topic: `robot/battery/status`):
  ```json
  {"soc": 85.2, "voltage": 12.56, "current": -1.45, "charging": 0}
  ```
- **Nested format** (Default topic: `robot/electrical/battery`):
  ```json
  {"data": {"soc_pct": 85.2, "voltage": 12.56, "current": -1.45}}
  ```

---

## Usage Guide

You can run the script globally:
```

### Running the Logger

To run the logger manually and store data to SQLite (default):
```bash
python3 battery_logger.py
```

To log **directly to CSV** instead of SQLite:
```bash
python3 battery_logger.py --use-csv
```

To configure a custom broker, port, or topics:
```bash
python3 battery_logger.py --broker 192.168.1.100 --port 1883 --topics robot/battery/status custom/battery
```

---

## Running in the Background (systemd)

To ensure the logger starts automatically on Pi 5 boot and restarts if it fails:

1. Copy the systemd service file to `/etc/systemd/system/`:
   ```bash
   sudo cp battery_logger.service /etc/systemd/system/
   ```

2. Reload systemd and enable/start the service:
   ```bash
   sudo systemctl daemon-reload
   sudo systemctl enable battery_logger.service
   sudo systemctl start battery_logger.service
   ```

3. To check logs and service status:
   ```bash
   sudo systemctl status battery_logger.service
   journalctl -u battery_logger.service -n 50 -f
   ```

---

## Downloading & Exporting Data

SQLite is highly recommended to protect logging data in case of sudden power cuts. You can download the `.db` file or export it to CSV on-demand before downloading.

### Export SQLite to CSV
To generate a CSV file from the SQLite database:
```bash
python3 battery_logger.py --export battery_telemetry.csv
```

---

## Telemetry Visualization

To plot the recorded telemetry and save it as a high-quality visualization (`battery_plot.png`):

### Plotting from SQLite Database (Default)
```bash
python3 plot_battery.py --db-path battery_telemetry.db --out battery_plot.png
```

### Plotting from CSV File
```bash
python3 plot_battery.py --csv-path battery_telemetry.csv --out battery_plot.png
```

The resulting `battery_plot.png` will show three stacked subplots over time:
1. **State of Charge (SoC %)**
2. **Voltage (V)**
3. **Current (A)** (with charging current vs load draw reference line)
