# Robot-to-Robot Communication System

## Overview

This subsystem enables bidirectional wireless communication between two robots using the nRF24L01+ transceiver. Each robot can access the other robot's sensor data (RobotPose, FieldBall, and Opponents) as if they were local sensor readings.

## Features

- **Real-time Pose Sharing**: Exchange precise position and heading information between robots
- **Ball Detection Sharing**: Each robot sends its local ball detection to the other robot
- **Opponent Detection Sharing**: Share up to 5 detected opponents with the other robot
- **Robot Selection via Button**: Use Button 2 to switch between Robot 1 and Robot 2
- **Efficient Binary Protocol**: Optimized data structures minimize payload size
- **Dynamic Payload Support**: Uses nRF24L01+ dynamic payload feature for variable-length packets
- **Statistics Tracking**: Monitor packet transmission/reception and failures

## Hardware Setup

### nRF24L01+ Wiring (Teensy 4.1)

Connect the nRF24L01+ module to the Teensy 4.1 following the SPI pinout:

```
nRF24L01+    →    Teensy 4.1
GND          →    GND
+3.3V        →    +3.3V (with 10µF capacitor close to module)
MOSI (DIN)   →    Pin 11 (MOSI)
MISO (DO)    →    Pin 12 (MISO)
SCK          →    Pin 13 (SCK)
CE           →    Pin 34
CSN          →    Pin 36
IRQ          →    (optional, not used in this implementation)
```

### Important Notes

1. **Power Supply**: The nRF24L01+ draws significant current (~15mA). Use a separate 3.3V regulator with a 10µF capacitor as close as possible to the module's power pins
2. **Antenna**: Ensure the antenna is properly connected for best range
3. **Spacing**: Keep the module away from high-speed digital lines to avoid noise

## Configuration

### Radio Settings (in communication.cpp)

The following settings are optimized for reliability and efficiency:

```
Data Rate:       250 kbps (slower = more reliable over distance)
Power Level:     RF24_PA_LOW (adjust to RF24_PA_HIGH for longer range)
Channel:         76 (out of 0-125, chosen to avoid WiFi interference)
CRC:             16-bit
Dynamic Payloads: Enabled (allows variable-length packets)
```

### Adjusting for Range

If robots need to communicate over longer distances:

1. Increase power level: Change `RF24_PA_LOW` to `RF24_PA_HIGH`
2. Reduce data rate: Change to `RF24_1MBPS` or `RF24_2MBPS` (more prone to errors)
3. Optimize antenna positioning
4. Add amplifier if necessary

## Data Protocol

### Packet Types

All communication packets follow a type-length-value format with 32-byte maximum payload:

**Type 0: Robot Pose (21 bytes)**
- Valid flag (1 byte)
- X position in mm (4 bytes, float)
- Y position in mm (4 bytes, float)
- Heading in degrees (4 bytes, float)
- Quality metric (4 bytes, float)
- Timestamp in ms (4 bytes, uint32_t)

**Type 1: Field Ball (21 bytes)**
- Valid flag (1 byte)
- X position in mm (4 bytes, float)
- Y position in mm (4 bytes, float)
- Distance to ball in cm (4 bytes, float)
- Angle to ball in degrees (4 bytes, float)
- Timestamp in ms (4 bytes, uint32_t)

**Type 2: Opponents (up to 85 bytes)**
- Number of opponents (1 byte)
- Up to 5 opponent records (17 bytes each):
  - Valid flag (1 byte)
  - X position in mm (4 bytes, float)
  - Y position in mm (4 bytes, float)
  - Confidence (4 bytes, float)
  - Timestamp in ms (4 bytes, uint32_t)

### Transmission Rates

- **Pose**: 20 Hz (50 ms interval)
- **Ball**: 20 Hz (50 ms interval)
- **Opponents**: 10 Hz (100 ms interval)

These rates can be adjusted in `communication.cpp` via:
- `POSE_SEND_INTERVAL_MS`
- `BALL_SEND_INTERVAL_MS`
- `OPPONENTS_SEND_INTERVAL_MS`

## Usage

### Basic Integration

The subsystem is automatically initialized in `robot.cpp`:

```cpp
// In setup()
initCommunication();

// In systemTick()
updateCommunication();
```

### Accessing Remote Robot Data

In your strategy or other subsystems, access data from the other robot the same way you access local data:

```cpp
#include "include/subsystems/communication.h"

// Get the other robot's pose
RobotPose remotePose;
getRemoteRobotPose(remotePose);
if (remotePose.valid) {
  Serial.print("Other robot at X=");
  Serial.print(remotePose.xMm);
  Serial.print(", Y=");
  Serial.println(remotePose.yMm);
}

// Get the other robot's ball detection
FieldBall remoteBall;
getRemoteFieldBall(remoteBall);

// Get the other robot's detected opponents
OpponentRobot remoteOpponents[5];
int opponentCount = 0;
getRemoteOpponents(remoteOpponents, 5, opponentCount);

// Get current robot number (1 or 2)
uint8_t robotNum = getCurrentRobotNumber();
```

### Monitoring Communication Health

Print communication statistics to the serial console:

```cpp
printCommunicationStats();
// Output: "Communication Stats - Robot: 1 | TX: 1234 | RX: 5678 | Failures: 0"
```

## Button Controls

### Button 2 - Robot Selection

- **Not Pressed**: Device operates as Robot 1
- **Pressed**: Device operates as Robot 2

The robot number can be checked at any time with `getCurrentRobotNumber()`.

### Button 1 - Robot Enable/Disable (unchanged)

This is handled by the existing system and still functions normally for starting/stopping the robot.

## Troubleshooting

### No Communication Between Robots

1. **Check Hardware**:
   - Verify nRF24L01+ is properly wired
   - Confirm 3.3V power is stable (use multimeter)
   - Check that antenna is connected
   - Verify SPI clock (Pin 13) is active

2. **Check Configuration**:
   - Both robots must use the same channel (76)
   - Both robots must use the same data rate (250kbps)
   - Verify pipes are correctly configured for Robot 1 ↔ Robot 2

3. **Serial Monitor Debugging**:
   - Open Serial Monitor at 115200 baud
   - Look for initialization message: "RF24 initialized. Robot: X"
   - Call `printCommunicationStats()` to check packet counters
   - High "Failures" count indicates transmission problems

### Poor Range

1. Check antenna connection
2. Reduce data rate to 1Mbps or 2Mbps
3. Increase power level to RF24_PA_HIGH
4. Move robots closer together to test basic communication first
5. Ensure antenna is positioned vertically and away from metal/high-speed digital lines

### Data Too Old

If `timestampMs` is significantly older than `millis()`:
- Increase transmission rates (reduce interval values)
- Check if other robot is sending data (use Serial Monitor)
- Verify radio is not spending too much time in transmit mode (should alternate TX/RX)

## Performance Notes

### Latency

With current settings (50ms pose/ball, 100ms opponents):
- Expected latency: ~50-200ms depending on radio traffic
- Both robots transmit, so there's inevitable congestion
- If lower latency is needed, increase data rates or reduce transmission frequency for non-critical data

### Power Consumption

The nRF24L01+ is active during `updateCommunication()`. It alternates between:
- **RX mode**: ~13mA (waiting for incoming data)
- **TX mode**: ~11mA (sending data)

Average consumption is roughly 12-14mA. For battery-powered robots, consider reducing transmission frequencies.

### Bandwidth

With maximum payload (32 bytes) at all transmission rates:
- Pose: 32 bytes × 20 Hz = 640 bytes/sec
- Ball: 32 bytes × 20 Hz = 640 bytes/sec
- Opponents: 32 bytes × 10 Hz = 320 bytes/sec
- **Total: ~1600 bytes/sec** (modest compared to nRF24's ~30 kbps capability)

## Advanced Configuration

### Changing Robot Identifiers

To use a different scheme for robot selection, modify `updateRobotSelection()` in `communication.cpp`. For example, to toggle on each button press:

```cpp
static unsigned long lastToggleMs = 0;
if (now - lastToggleMs > BUTTON_DEBOUNCE_MS && digitalRead(button2) == HIGH) {
  currentRobotNumber = (currentRobotNumber == 1) ? 2 : 1;
  lastToggleMs = now;
}
```

### Adjusting Data Retention

The received data is cached in global structures. To clear stale data automatically:

```cpp
// Add this to updateCommunication()
unsigned long dataAge = millis() - remoteRobotPose.timestampMs;
if (dataAge > 1000) {  // 1 second timeout
  remoteRobotPose.valid = false;
}
```

### Custom Message Types

To add new message types (e.g., strategy state):
1. Add a new struct definition in `communication.cpp`
2. Create serialize/deserialize functions
3. Add a new type (3+) in the transmit/receive switch statements
4. Update transmission interval if needed

## Files Modified

- `src/include/subsystems/communication.h` - Public interface
- `src/cpp/subsystems/communication.cpp` - Implementation with RF24 driver
- `src/robot.cpp` - Integration into main robot code (added `initCommunication()` and `updateCommunication()`)

## Future Enhancements

- **Acknowledgments**: Implement packet acknowledgment for guaranteed delivery
- **Error Correction**: Add forward error correction for unreliable links
- **Adaptive Rates**: Automatically adjust transmission frequency based on link quality
- **Multiple Opponents**: Extend to support more than 5 opponents if needed
- **Strategy Communication**: Add ability to send strategy commands or game state between robots
- **Power Modes**: Implement sleep modes for battery conservation
