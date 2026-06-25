#!/usr/bin/env python3
"""
Battery Telemetry Plotter
Reads battery telemetry from SQLite or CSV and generates a PNG plot.
"""

import argparse
import csv
import os
import sqlite3
import sys
from datetime import datetime
import matplotlib.pyplot as plt
import matplotlib.dates as mdates

def read_from_sqlite(db_path):
    """Fetch telemetry data from SQLite database."""
    db_path = os.path.abspath(db_path)
    if not os.path.exists(db_path):
        print(f"[!] SQLite DB not found: {db_path}")
        return None

    try:
        conn = sqlite3.connect(db_path)
        cursor = conn.cursor()
        cursor.execute("SELECT timestamp, soc, voltage, current, power FROM battery_telemetry ORDER BY timestamp ASC")
        rows = cursor.fetchall()
        conn.close()
        
        data = {
            "timestamps": [],
            "soc": [],
            "voltage": [],
            "current": [],
            "power": []
        }
        
        for r in rows:
            # Parse ISO 8601 timestamp
            try:
                dt = datetime.fromisoformat(r[0])
            except ValueError:
                # Try parsing with "Z" at the end if present
                ts = r[0].replace("Z", "+00:00")
                dt = datetime.fromisoformat(ts)
            
            data["timestamps"].append(dt)
            data["soc"].append(r[1])
            data["voltage"].append(r[2])
            data["current"].append(r[3])
            data["power"].append(r[4])
            
        return data
    except Exception as e:
        print(f"[!] Error reading SQLite: {e}")
        return None

def read_from_csv(csv_path):
    """Fetch telemetry data from CSV file."""
    csv_path = os.path.abspath(csv_path)
    if not os.path.exists(csv_path):
        print(f"[!] CSV file not found: {csv_path}")
        return None

    try:
        data = {
            "timestamps": [],
            "soc": [],
            "voltage": [],
            "current": [],
            "power": []
        }
        
        with open(csv_path, mode='r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                try:
                    dt = datetime.fromisoformat(row["timestamp"].replace("Z", "+00:00"))
                except ValueError:
                    continue
                
                data["timestamps"].append(dt)
                data["soc"].append(float(row["soc"]) if row["soc"] else None)
                data["voltage"].append(float(row["voltage"]) if row["voltage"] else None)
                data["current"].append(float(row["current"]) if row["current"] else None)
                # Power might not exist in old logs or can be calculated
                p = row.get("power")
                if p:
                    data["power"].append(float(p))
                elif row["voltage"] and row["current"]:
                    data["power"].append(float(row["voltage"]) * float(row["current"]))
                else:
                    data["power"].append(None)
                    
        return data
    except Exception as e:
        print(f"[!] Error reading CSV: {e}")
        return None

def plot_telemetry(data, output_image):
    """Generate a 3-panel plot showing SoC, Voltage, and Current."""
    if not data or not data["timestamps"]:
        print("[!] No data to plot.")
        return

    # Filter out None values for clean plotting
    times = data["timestamps"]
    
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    
    # Plot SoC
    soc_vals = [x for x in data["soc"] if x is not None]
    soc_times = [times[i] for i, x in enumerate(data["soc"]) if x is not None]
    if soc_vals:
        ax1.plot(soc_times, soc_vals, color='#1f77b4', linewidth=2, label='SoC (%)')
        ax1.set_ylabel('SoC (%)', color='#1f77b4')
        ax1.tick_params(axis='y', labelcolor='#1f77b4')
        ax1.set_ylim(-5, 105)
        ax1.grid(True, linestyle='--', alpha=0.5)
        ax1.set_title('Battery Telemetry Over Time')
    
    # Plot Voltage
    volt_vals = [x for x in data["voltage"] if x is not None]
    volt_times = [times[i] for i, x in enumerate(data["voltage"]) if x is not None]
    if volt_vals:
        ax2.plot(volt_times, volt_vals, color='#ff7f0e', linewidth=2, label='Voltage (V)')
        ax2.set_ylabel('Voltage (V)', color='#ff7f0e')
        ax2.tick_params(axis='y', labelcolor='#ff7f0e')
        ax2.grid(True, linestyle='--', alpha=0.5)
        
    # Plot Current
    curr_vals = [x for x in data["current"] if x is not None]
    curr_times = [times[i] for i, x in enumerate(data["current"]) if x is not None]
    if curr_vals:
        ax3.plot(curr_times, curr_vals, color='#2ca02c', linewidth=2, label='Current (A)')
        ax3.set_ylabel('Current (A)', color='#2ca02c')
        ax3.tick_params(axis='y', labelcolor='#2ca02c')
        ax3.grid(True, linestyle='--', alpha=0.5)
        
        # Add horizontal line at 0A reference
        ax3.axhline(0, color='red', linestyle=':', alpha=0.5)

    # Format timestamps on X-axis
    locator = mdates.AutoDateLocator()
    formatter = mdates.DateFormatter('%m-%d %H:%M')
    ax3.xaxis.set_major_locator(locator)
    ax3.xaxis.set_major_formatter(formatter)
    fig.autofmt_xdate()
    
    plt.xlabel('Time (MM-DD HH:MM)')
    plt.tight_layout()
    
    plt.savefig(output_image, dpi=150)
    print(f"[+] Plot successfully saved to {output_image}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Battery Telemetry Plotter")
    parser.add_argument("--db-path", default="battery_telemetry.db", help="Path to SQLite database")
    parser.add_argument("--csv-path", help="Path to CSV file (if using CSV instead of SQLite)")
    parser.add_argument("--out", default="battery_plot.png", help="Path to output plot image (default: battery_plot.png)")
    
    args = parser.parse_args()
    
    if args.csv_path:
        data = read_from_csv(args.csv_path)
    else:
        data = read_from_sqlite(args.db_path)
        
    if data:
        plot_telemetry(data, args.out)
    else:
        sys.exit(1)
