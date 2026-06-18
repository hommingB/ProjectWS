# Pi 5 AP+STA + MQTT + ROS2 Configuration Summary

> Generated: 2026-06-18
> Context: Raspberry Pi 5 with built-in Wi-Fi (CYW43455) running AP+STA mode, MQTT broker, and ROS2 stack.

---

## 1. Network Architecture

```
┌─────────────────────────────────────────┐
│           Raspberry Pi 5                │
│  ┌─────────┐      ┌─────────────────┐  │
│  │ wlan0   │─────▶│ S20 FE hotspot  │  │  (STA - internet)
│  │ 172.x   │      │ (upstream WiFi)   │  │
│  └─────────┘      └─────────────────┘  │
│       ▲                                 │
│       │ NAT / IP Forwarding             │
│       ▼                                 │
│  ┌─────────┐      ┌─────────────────┐  │
│  │ uap0    │◀────│ Devices         │  │  (AP - PiLocalNet)
│  │ 192.168.│      │ (192.168.4.x)   │  │
│  │ 4.1/24  │      │                 │  │
│  └─────────┘      └─────────────────┘  │
└─────────────────────────────────────────┘
```

---

## 2. Key Configuration Files

### 2.1 AP Interface Creation
**File:** `/etc/systemd/system/create-uap0.service`

```ini
[Unit]
Description=Create uap0 interface
After=systemd-networkd.service
Wants=systemd-networkd.service

[Service]
Type=oneshot
ExecStart=/sbin/iw dev wlan0 interface add uap0 type __ap
ExecStartPost=/sbin/iw dev wlan0 set power_save off
ExecStartPost=/sbin/iw dev uap0 set power_save off
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
```

### 2.2 AP Network (uap0)
**File:** `/etc/systemd/network/10-uap0.network`

```ini
[Match]
Name=uap0

[Network]
Address=192.168.4.1/24
DHCPServer=yes
IPForward=yes

[DHCPServer]
PoolOffset=10
PoolSize=20
EmitDNS=yes
DNS=192.168.4.1
DefaultLeaseTimeSec=86400
MaxLeaseTimeSec=604800

[Link]
RequiredForOnline=no
```

### 2.3 STA Network (wlan0)
**File:** `/etc/systemd/network/20-wlan0.network`

```ini
[Match]
Name=wlan0

[Network]
DHCP=yes
IPForward=yes

[DHCP]
RouteMetric=100
UseDNS=yes
```

### 2.4 Hostapd (AP Config)
**File:** `/etc/hostapd/hostapd.conf`

```ini
interface=uap0
driver=nl80211
ssid=PiLocalNet
hw_mode=g
channel=11              # Must match upstream WiFi channel
wmm_enabled=1
macaddr_acl=0
auth_algs=1
ignore_broadcast_ssid=0
wpa=2
wpa_passphrase=YourSecurePassword
wpa_key_mgmt=WPA-PSK
wpa_pairwise=CCMP
rsn_pairwise=CCMP
beacon_int=100
dtim_period=2
max_num_sta=8
ap_max_inactivity=300
```

**File:** `/etc/default/hostapd`
```bash
DAEMON_CONF="/etc/hostapd/hostapd.conf"
```

### 2.5 DHCP Server (dnsmasq)
**File:** `/etc/dnsmasq.conf`

```conf
interface=uap0
bind-interfaces
dhcp-range=192.168.4.10,192.168.4.30,255.255.255.0,24h
domain-needed
bogus-priv
dhcp-authoritative
dhcp-leasefile=/tmp/dnsmasq.leases
except-interface=lo
bind-dynamic
```

### 2.6 NAT / IP Forwarding

Enable forwarding:
```bash
# /etc/sysctl.d/99-ipforward.conf
net.ipv4.ip_forward=1
```

iptables rules:
```bash
sudo iptables -t nat -A POSTROUTING -o wlan0 -j MASQUERADE
sudo iptables -A FORWARD -i uap0 -o wlan0 -j ACCEPT
sudo iptables -A FORWARD -i wlan0 -o uap0 -m state --state RELATED,ESTABLISHED -j ACCEPT
```

Save rules:
```bash
sudo apt install iptables-persistent -y
sudo netfilter-persistent save
```

### 2.7 Mosquitto MQTT Broker
**File:** `/etc/mosquitto/mosquitto.conf`

```conf
# Localhost - for robot_manager and local tools
listener 1883 127.0.0.1
allow_anonymous true

# PiLocalNet - for ESP32 and devices
listener 1883 192.168.4.1
allow_anonymous true

# Persistence
persistence true
persistence_location /var/lib/mosquitto/

# Logging
log_dest file /var/log/mosquitto/mosquitto.log
log_type error
log_type warning
log_type information

# Bridge to upstream broker (optional)
connection bridge-to-upstream
address <UPSTREAM_IP>:1883
topic # both 0
bridge_protocol_version mqttv311
try_private false
start_type automatic
cleansession true
notifications false

# WebSockets (optional)
listener 9001 0.0.0.0
protocol websockets
```

---

## 3. Services Enabled

```bash
sudo systemctl enable systemd-networkd
sudo systemctl enable create-uap0.service
sudo systemctl enable hostapd
sudo systemctl enable dnsmasq
sudo systemctl enable mosquitto
```

---

## 4. Known Issues & Fixes

### 4.1 Boot Slow (>1 min)
**Cause:** `systemd-networkd-wait-online.service` and `NetworkManager-wait-online.service` timeout.

**Fix:**
```bash
sudo systemctl disable systemd-networkd-wait-online.service
sudo systemctl mask systemd-networkd-wait-online.service
sudo systemctl disable NetworkManager-wait-online.service
sudo systemctl mask NetworkManager-wait-online.service
```

### 4.2 Samsung Tablet Disconnect
**Cause:** Wi-Fi power save, Smart Network Switch, aggressive scanning.

**Fix on Pi:**
- Disable power save on wlan0/uap0
- Fixed channel matching upstream
- Longer DHCP lease times

**Fix on Samsung tablet:**
- Settings → Connections → Wi-Fi → Advanced → Smart Network Switch → OFF
- Settings → Developer Options → Wi-Fi Scan Throttling → OFF
- Settings → Connections → Wi-Fi → Advanced → Keep Wi-Fi on during sleep → Always

### 4.3 DNS Resolution (GitHub access)
**Cause:** `systemd-resolved` disabled, no nameserver configured.

**Fix:**
```bash
# Option A: Manual resolv.conf
sudo rm -f /etc/resolv.conf
echo "nameserver 8.8.8.8" | sudo tee /etc/resolv.conf
echo "nameserver 8.8.4.4" | sudo tee -a /etc/resolv.conf

# Option B: Re-enable systemd-resolved (safe, does not affect Wi-Fi)
sudo systemctl enable systemd-resolved
sudo systemctl start systemd-resolved
sudo ln -sf /run/systemd/resolve/stub-resolv.conf /etc/resolv.conf
```

### 4.4 ROS2 Discovery Across Subnets
**Cause:** Multicast discovery does not work across NAT/subnets.

**Fix with Cyclone DDS:**

Create `cyclonedds.xml` on **both** Pi and computer:

```xml
<?xml version="1.0" encoding="UTF-8"?>
<CycloneDDS xmlns="https://cdds.io/config">
  <Domain id="2">
    <General>
      <Interfaces>
        <NetworkInterface name="uap0" priority="default" multicast="true"/>
      </Interfaces>
      <AllowMulticast>spdp</AllowMulticast>
    </General>
    <Discovery>
      <Peers>
        <Peer address="192.168.4.1"/>
        <Peer address="192.168.4.10"/>
      </Peers>
      <ParticipantIndex>auto</ParticipantIndex>
    </Discovery>
  </Domain>
</CycloneDDS>
```

Export:
```bash
export ROS_DOMAIN_ID=2
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/tuanpi5/cyclonedds.xml
```

---

## 5. robot_manager.service

**File:** `/etc/systemd/system/robot_manager.service`

```ini
[Unit]
Description=Robot Manager — ROS2 stack + MQTT lifecycle
After=mosquitto.service
Wants=mosquitto.service

[Service]
Type=simple
User=tuanpi5
WorkingDirectory=/home/tuanpi5
ExecStart=/usr/bin/python3 /home/tuanpi5/robot_manager.py
Restart=on-failure
RestartSec=10
TimeoutStopSec=40
StandardOutput=journal
StandardError=journal
SyslogIdentifier=robot_manager

[Install]
WantedBy=multi-user.target
```

**Prerequisites:**
```bash
# Passwordless sudo for shutdown
sudo visudo
# Add: tuanpi5 ALL=(ALL) NOPASSWD: /sbin/shutdown

# Python dependencies
pip3 install paho-mqtt
```

---

## 6. Hardware Limitations

| Limitation | Details |
|---|---|
| Single radio | AP and STA share same physical Wi-Fi chip |
| Same channel | Both must use same frequency/channel (e.g., ch 11) |
| Same band | Cannot mix 2.4GHz and 5GHz |
| Bandwidth shared | Throughput split between AP and STA |
| True L2 bridge | Not possible with single radio; use NAT instead |

---

## 7. Useful Commands

```bash
# Check interfaces
ip -br addr show
iw dev

# Check services
sudo systemctl status hostapd dnsmasq mosquitto

# Check Wi-Fi clients
sudo iw dev uap0 station dump

# Check DHCP leases
cat /tmp/dnsmasq.leases

# Check NAT
sudo iptables -t nat -L -n -v
sudo iptables -L FORWARD -n -v

# Check MQTT
mosquitto_sub -h localhost -t test -v
mosquitto_pub -h localhost -t test -m "hello"

# View logs
journalctl -u hostapd -f
journalctl -u dnsmasq -f
journalctl -u robot_manager -f

# Boot time analysis
systemd-analyze
systemd-analyze blame
```

---

## 8. File Locations Summary

| File | Purpose |
|---|---|
| `/etc/hostapd/hostapd.conf` | AP SSID, password, channel |
| `/etc/default/hostapd` | Points to config file |
| `/etc/dnsmasq.conf` | DHCP range, lease file |
| `/etc/systemd/network/10-uap0.network` | Static IP for uap0 |
| `/etc/systemd/network/20-wlan0.network` | DHCP client for wlan0 |
| `/etc/systemd/system/create-uap0.service` | Creates virtual AP interface |
| `/etc/mosquitto/mosquitto.conf` | MQTT broker + bridge config |
| `/etc/sysctl.d/99-ipforward.conf` | IP forwarding toggle |
| `/etc/systemd/system/robot_manager.service` | ROS2 stack launcher |
| `/home/tuanpi5/robot_manager.py` | Main Python script |
| `/home/tuanpi5/cyclonedds.xml` | DDS discovery config |
