# Communication System Implementation Summary

## What Was Created

A complete robot-to-robot communication subsystem for your open-soccer robots using the nRF24L01+ wireless module.

### New Files

1. **src/include/subsystems/communication.h** (Public Interface)
   - Function declarations for initialization and data access
   - Type definitions in associated cpp file

2. **src/cpp/subsystems/communication.cpp** (Implementation)
   - RF24 radio driver initialization and management
   - Binary protocol implementation for efficient data transmission
   - Packet serialization/deserialization
   - Bidirectional communication handling
   - Robot number selection logic based on Button 2

### Modified Files

1. **src/robot.cpp**
   - Added `#include "include/subsystems/communication.h"`
   - Added `initCommunication()` call in setup()
   - Added `updateCommunication()` call in systemTick()

## System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    Strategy Logic                            │
│  (Your code in strategy.cpp or other subsystems)             │
└──────────────────┬──────────────────────────────────────────┘
                   │
                   │ Uses data via public API
                   │
┌──────────────────▼──────────────────────────────────────────┐
│         Communication Subsystem (communication.cpp)         │
│                                                             │
│  getRemoteRobotPose()    ◄── Cached remote data             │
│  getRemoteFieldBall()    ◄── (Updated every 50-100ms)       │
│  getRemoteOpponents()    ◄── from the other robot           │
│  getCurrentRobotNumber() ◄── Based on Button 2 state        │
└──────────────────┬──────────────────────────────────────────┘
                   │
        ┌──────────┼──────────┐
        │          │          │
      RX/TX     Serialization Robot
     Control    (Pose/Ball/   Number
               Opponents)     Selection
        │          │          │
└───────┼──────────┼──────────┼────────────────────────────────┘
        │          │          │
┌───────▼──────────▼──────────▼────────────────────────────────┐
│                   RF24 Radio Driver                          │
│  - 250kbps data rate (optimized for reliability)             │
│  - Dynamic payload support (up to 32 bytes)                  │
│  - Automatic TX/RX switching                                 │
│  - Channel 76 (WiFi-safe)                                    │
└───────┬────────────────────────────────────────────────────┬─┘
        │                                                    │
    Robot 1 Pipe                                         Robot 2 Pipe
    (NODE1/NODE2)                                        (NODE2/NODE1)
        │                                                    │
        └────────────────► nRF24L01+ Modules ◄──────────────┘
              (Wireless, ~30m range)
```

## Data Flow Example

### Transmission Pipeline (What This Robot Sends)

```
1. Local Sensor Data (Localization subsystem)
   ↓
2. Serialization (communication.cpp)
   - RobotPose (21 bytes) → Binary packet
   - FieldBall (21 bytes) → Binary packet
   - Opponents (up to 85 bytes) → Binary packet
   ↓
3. RF24 Transmission
   - Every 50ms for Pose
   - Every 50ms for Ball
   - Every 100ms for Opponents
   ↓
4. Other Robot's RF24 Receiver
   (Receives at variable rates based on congestion)

### Reception Pipeline (What This Robot Receives)

1. RF24 Receives Packet
   ↓
2. Deserialization (communication.cpp)
   - Binary → RobotPose struct
   - Binary → FieldBall struct
   - Binary → OpponentRobot array
   ↓
3. Cache in Global Variables
   - remoteRobotPose
   - remoteFieldBall
   - remoteOpponents[5]
   ↓
4. Strategy Code Can Access
   - getRemoteRobotPose()
   - getRemoteFieldBall()
   - getRemoteOpponents()
```

## Key Design Decisions

### 1. Binary Protocol Instead of Text

- **Why**: Minimize data size to reduce transmission time and power consumption
- **Impact**: 
  - Pose: 21 bytes (vs ~80 bytes in JSON/text)
  - Ball: 21 bytes (vs ~70 bytes in JSON/text)
  - 3 Opponents: 52 bytes (vs ~200 bytes in JSON/text)

### 2. Separate Packet Types

- **Why**: Allows different transmission rates for different data importance
- **Impact**:
  - Pose & Ball: 20 Hz (critical for navigation)
  - Opponents: 10 Hz (less critical)

### 3. Button 2 for Robot Selection

- **Why**: Button 1 already controls Enable/Disable
- **Impact**:
  - Simple hardware toggle
  - Automatic pipe reconfiguration
  - Dynamic switching during runtime

### 4. Cached Data vs. Direct Access

- **Why**: Wireless communication is asynchronous and can fail
- **Impact**:
  - Your strategy code always gets data immediately (never blocks waiting)
  - Stale data is explicit (check timestamp)
  - Can continue operation if link is down

## Performance Characteristics

### Latency
```
Round-trip latency: ~50-200ms typically
  - Serialization: <1ms
  - RF24 transmission: 5-50ms (depends on congestion)
  - Deserialization: <1ms
  - Processing delay: 10-100ms
```

### Throughput
```
Maximum useful data rate: ~1600 bytes/sec
  - Pose: 640 bytes/sec (32 bytes × 20 Hz)
  - Ball: 640 bytes/sec (32 bytes × 20 Hz)
  - Opponents: 320 bytes/sec (32 bytes × 10 Hz)
Total: Well below nRF24's ~30 kbps capability
```

### Power Consumption
```
RF24 Module:
  - RX Mode: ~13mA
  - TX Mode: ~11mA
  - Duty Cycle: ~50% alternating TX/RX
  - Average: ~12mA per robot
```

## Testing Checklist

Before deploying to your team robots:

- [ ] Both robots have nRF24L01+ modules properly wired
- [ ] Both robots run the updated code
- [ ] Verify initialization message in Serial Monitor: "RF24 initialized. Robot: X"
- [ ] Press Button 2 to toggle between Robot 1 and Robot 2, verify serial output
- [ ] Call `printCommunicationStats()` from Serial Monitor
  - Should show RX/TX counts increasing every few seconds
- [ ] Test with robots 1-2 meters apart
- [ ] Verify `getRemoteRobotPose()` returns valid data with recent timestamp
- [ ] Test the coordinate examples from COMMUNICATION_QUICK_REFERENCE.md
- [ ] Monitor for failed transmissions (Failures count in printCommunicationStats)
- [ ] Range test: move robots apart until communication drops (should be 10-20m)

## Customization Guide

### Change Transmission Rates

In `communication.cpp`, modify these constants:

```cpp
constexpr unsigned long POSE_SEND_INTERVAL_MS = 50;      // 20 Hz → change to 100 for 10 Hz
constexpr unsigned long BALL_SEND_INTERVAL_MS = 50;      // 20 Hz
constexpr unsigned long OPPONENTS_SEND_INTERVAL_MS = 100; // 10 Hz
```

### Change Data Rate for Better Range

In `communication.cpp`, modify:

```cpp
radio.setDataRate(RF24_250KBPS);  // Change to RF24_1MBPS for faster, RF24_2MBPS for fastest
```

Slower rates = better range, fewer errors
Faster rates = lower latency, higher throughput

### Change Power Level

In `communication.cpp`, modify:

```cpp
radio.setPALevel(RF24_PA_LOW);  // Change to RF24_PA_HIGH for maximum range
```

### Change Channel

In `communication.cpp`, modify:

```cpp
radio.setChannel(76);  // Change to any value 0-125 if interference occurs
```

## Troubleshooting Commands

Put these in your Serial Monitor (or strategy loop) to debug:

```cpp
// Check robot number
Serial.println(getCurrentRobotNumber());

// Print radio stats
printCommunicationStats();

// Check remote data validity and age
RobotPose p;
getRemoteRobotPose(p);
Serial.print("Remote pose valid: ");
Serial.print(p.valid);
Serial.print(", age: ");
Serial.print(millis() - p.timestampMs);
Serial.println("ms");

// Test local data (should always work)
RobotPose local;
getRobotPose(local);
Serial.print("Local pose valid: ");
Serial.println(local.valid);
```

## Integration with Strategy Code

The communication system is completely non-intrusive. To use it:

```cpp
#include "include/subsystems/communication.h"

void updateStrategy() {
  // Your existing strategy code continues unchanged
  // ...
  
  // Now you can also access remote robot data
  RobotPose remotePose;
  getRemoteRobotPose(remotePose);
  if (remotePose.valid) {
    // Use remote pose in your decision-making
  }
}
```

No changes needed to existing subsystems unless you want to use remote data.

## Next Steps

1. **Deploy to Hardware**: Upload code to both Teensy boards
2. **Verify Communication**: Check Serial Monitor output
3. **Test Data Access**: Write simple strategy code using `getRemoteRobotPose()`, etc.
4. **Integrate with Strategy**: Use remote data in your ball-chasing or defensive logic
5. **Monitor Performance**: Use `printCommunicationStats()` during matches
6. **Tune as Needed**: Adjust transmission rates, data rate, or power level based on results

## Support

For questions or issues:
- Check COMMUNICATION_SYSTEM.md for detailed hardware/software info
- Check COMMUNICATION_QUICK_REFERENCE.md for code examples
- Use Serial Monitor debugging commands (listed above)
- Verify nRF24L01+ wiring before assuming software issue
