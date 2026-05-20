# MQTT Broker Architecture

## Overview

This document describes the MQTT broker architecture for the ROS2 MQTT bridge running on a Raspberry Pi 5.

## Architecture Diagram

```
                         Main/Remote Broker
                         (cloud/server)
                              │
                              │ MQTT Bridge (TLS optional)
                              ▼
┌─────────────┐    ┌─────────────────┐    ┌─────────────────────┐
│   Local      │◄───│  mosquitto.conf │───►│   mqtt_bridge_node   │
│   Broker     │    │    (bridge)     │    │   (connects locally) │
│ (mosquitto)  │    │                 │    │                     │
└──────────────┘    └─────────────────┘    └─────────────────────┘
    Pi 5                Pi 5                    Pi 5
    :1883               :1883                   :1883
```

## Components

### 1. Local Mosquitto Broker
- Runs on Raspberry Pi 5 (localhost:1883)
- Always available for local clients
- Bridges to main broker when available
- Handles message persistence during disconnection

### 2. Mosquitto Bridge
- Configured in `/etc/mosquitto/conf.d/bridge.conf`
- Bidirectional sync of `robot/#` topics
- Auto-reconnect on connection loss
- TLS support for secure connections

### 3. mqtt_bridge_node
- Connects to local broker (localhost:1883)
- Converts ROS2 ↔ MQTT messages
- Does NOT need to know about main broker

## Setup Instructions

### Prerequisites
- Raspberry Pi 5 running Raspberry Pi OS
- ROS2 Humble/ Jazzy installed
- Network access to main MQTT broker

### Installation Steps

1. **Run the broker setup script:**
   ```bash
   cd ~/ros2_project_ws/src/ros2_mqtt_bridge
   sudo ./scripts/broker_setup.sh --main-broker YOUR_MAIN_BROKER_IP
   ```

2. **For TLS-enabled main broker:**
   ```bash
   sudo ./scripts/broker_setup.sh --main-broker broker.example.com --tls
   ```

3. **Configure certificates** (if using TLS):
   ```bash
   sudo mkdir -p /etc/mosquitto/certs
   sudo cp ca.crt client.crt client.key /etc/mosquitto/certs/
   ```

4. **Update bridge.conf** with TLS settings:
   ```
   bridge_cafile /etc/mosquitto/certs/ca.crt
   bridge_certfile /etc/mosquitto/certs/client.crt
   bridge_keyfile /etc/mosquitto/certs/client.key
   ```

5. **Restart the broker:**
   ```bash
   sudo systemctl restart mosquitto-local.service
   ```

## Broker Management

### Useful Commands

```bash
# Check broker status
sudo systemctl status mosquitto-local.service

# View logs
sudo journalctl -u mosquitto-local.service -f

# Restart broker
sudo systemctl restart mosquitto-local.service

# Stop broker
sudo systemctl stop mosquitto-local.service

# Test local connection
mosquitto_pub -h localhost -t test -m "hello"

# Test subscription
mosquitto_sub -h localhost -t "robot/#"
```

## Configuration Files

| File | Location | Purpose |
|------|----------|---------|
| `bridge.conf` | `/etc/mosquitto/conf.d/bridge.conf` | Main bridge configuration |
| `mosquitto-local.service` | `/etc/systemd/system/` | Systemd service unit |
| `mqtt_bridge.launch.py` | Project | ROS2 launch with broker params |

## Parameters

### mqtt_bridge_node Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `mqtt_host` | `localhost` | MQTT broker host |
| `mqtt_port` | `1883` | MQTT broker port |
| `dock_x` | `0.0` | Dock position X |
| `dock_y` | `0.0` | Dock position Y |
| `dock_yaw` | `0.0` | Dock orientation |

### Bridge Configuration

| Setting | Value | Description |
|---------|-------|-------------|
| `connection` | `cloud-bridge` | Bridge name |
| `address` | `MAIN_BROKER_HOST:1883` | Remote broker address |
| `topic` | `robot/#` | Topics to bridge |
| `cleansession` | `false` | Persist subscriptions |
| `keepalive_interval` | `60` | Keepalive in seconds |

## Topic Mapping

### Bridged Topics

| Topic Pattern | Direction | Description |
|--------------|-----------|-------------|
| `robot/cmd/#` | in/out/both | Command topics |
| `robot/state/#` | in/out/both | State topics |
| `robot/electrical/battery` | in/out/both | Battery data |

## Troubleshooting

### Bridge Not Connecting

1. Check main broker accessibility:
   ```bash
   ping MAIN_BROKER_HOST
   telnet MAIN_BROKER_HOST 1883
   ```

2. Verify bridge configuration:
   ```bash
   sudo cat /etc/mosquitto/conf.d/bridge.conf
   ```

3. Check mosquitto logs:
   ```bash
   sudo journalctl -u mosquitto-local.service -n 50
   ```

### Local Clients Can't Connect

1. Verify broker is running:
   ```bash
   sudo systemctl status mosquitto-local.service
   ```

2. Check firewall:
   ```bash
   sudo ufw allow 1883
   ```

### TLS Connection Failures

1. Verify certificate files exist:
   ```bash
   ls -la /etc/mosquitto/certs/
   ```

2. Test certificate:
   ```bash
   openssl s_client -connect MAIN_BROKER_HOST:8883 -CAfile /etc/mosquitto/certs/ca.crt
   ```

## Security Considerations

1. **Change `allow_anonymous false`** in production and configure authentication
2. Use TLS for all connections to main broker
3. Store certificates with restricted permissions: `chmod 600 /etc/mosquitto/certs/*.key`
4. Consider using client certificates for mutual TLS

## Future Enhancements

- [ ] Add authentication (username/password)
- [ ] Implement TLS client certificate authentication
- [ ] Add bridge status monitoring via ROS2 topic
- [ ] Automatic main broker failover to backup broker