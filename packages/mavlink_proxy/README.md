# MAVLink Proxy Driver

MAVLink TCP-to-DTPS proxy driver for Duckietown robots.

## Overview

This driver bridges MAVLink TCP connections with the DTPS switchboard, enabling MAVLink-based autopilots (like PX4) to communicate with the Duckiematrix simulation engine.

## Architecture

```
PX4 SITL (tcp:4560) <--> MAVLinkProxyNode <--> Switchboard <--> Duckiematrix Engine
                                                                        |
                                                                  DD24RobotEntity
                                                                        |
                                                                  PX4Multirotor
                                                              (tcp:localhost:14560)
```

The driver follows the same pattern as other Duckietown drivers (IMU, LED, etc.):

1. Connects to PX4 SITL via TCP socket
2. Creates local DTPS queues for TX/RX
3. Exposes queues to the switchboard at standard paths
4. Engine's `robot_connector` automatically discovers and subscribes

## Topic Naming Convention

Following Duckietown conventions:

- **TX (Sensor)**: `/sensor/mavlink/{proxy_name}/tx`
  - Data flowing FROM PX4 TO engine
  - Contains telemetry, state updates, heartbeats

- **RX (Actuator)**: `/actuator/mavlink/{proxy_name}/rx`
  - Data flowing FROM engine TO PX4
  - Contains commands, setpoints, mission items

## Configuration

See `config/mavlink_proxy_mavlink.default.yaml`:

```yaml
px4_host: "localhost"
px4_port: 4560
buffer_size: 4096
```

## Usage

### Running the Driver

```bash
python3 main.py --config default --proxy-name mavlink
```

### With PX4 SITL

1. Start PX4 SITL with the tag matching your host architecture:
```bash
# amd64
docker run --rm --network host duckietown/dt-px4:ente-amd64

# arm64
docker run --rm --network host duckietown/dt-px4:ente-arm64v8
```

2. Start MAVLink Proxy:
```bash
python3 main.py --config default --proxy-name mavlink
```

3. Start Duckiematrix engine with DD24RobotEntity
   - The engine will automatically connect via `robot_connector`
   - DD24RobotEntity subscribes to switchboard paths

## Implementation Details

### Switchboard Exposure

```python
await (self.switchboard / "sensor" / "mavlink" / self.proxy_name / "tx").expose(tx_queue)
await (self.switchboard / "actuator" / "mavlink" / self.proxy_name / "rx").expose(rx_queue)
```

### Engine Integration

In `DD24RobotEntity`:

```python
mavlink_tx_path = Path(self.world_key) / "sensor" / "mavlink" / "mavlink" / "tx"
mavlink_rx_path = Path(self.world_key) / "actuator" / "mavlink" / "mavlink" / "rx"

self.world.declare_output(str(mavlink_tx_path), MAVLinkData)
self.world.declare_input(str(mavlink_rx_path), MAVLinkData)
```

The `robot_connector` automatically bridges these paths.

## Comparison with Other Drivers

| Feature | IMU Driver | LED Driver | **MAVLink Proxy** |
|---------|-----------|-----------|-------------------|
| Type | Sensor | Actuator | **Sensor + Actuator** |
| Switchboard Path | `/sensor/imu/...` | `/actuator/lights/...` | **`/sensor|actuator/mavlink/...`** |
| Data Flow | Device → Engine | Engine → Device | **Bidirectional** |
| Queue Count | 1+ (per stream) | 1+ (per pattern) | **2 (TX + RX)** |
| External Conn | I2C/Hardware | Hardware | **TCP Socket** |

## Benefits

1. **Standard Pattern**: Follows established Duckietown driver conventions
2. **Automatic Discovery**: Switchboard enables automatic connection
3. **Single DTPS Server**: Uses engine's WorldConnector
4. **Bidirectional**: Supports both telemetry and commands
5. **Non-blocking**: Async I/O for TCP and DTPS

## Migration from dt-tcp-to-dtps-publisher

The old `dt-tcp-to-dtps-publisher` created a separate DTPS server. This driver:

- ✅ Uses single DTPS server (engine's WorldConnector)
- ✅ Follows standard driver pattern
- ✅ Uses switchboard for automatic discovery
- ✅ Matches IMU/LED architecture
- ✅ Proper topic hierarchy

## Dependencies

- `dt-node-utils`: Node base class, switchboard access
- `dtps-http`: RawData, ObjectQueue
- Standard library: `socket`, `asyncio`
