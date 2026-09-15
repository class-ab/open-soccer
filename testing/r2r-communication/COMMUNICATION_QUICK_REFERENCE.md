# Communication System Quick Reference

## Usage Examples

### Example 1: Check if Both Robots Have the Ball

```cpp
#include "include/subsystems/communication.h"
#include "include/subsystems/localization.h"

void updateStrategy() {
  // Get local ball detection
  FieldBall localBall;
  getFieldBall(localBall);
  
  // Get remote robot's ball detection
  FieldBall remoteBall;
  getRemoteFieldBall(remoteBall);
  
  if (localBall.valid && remoteBall.valid) {
    Serial.println("Both robots see the ball!");
  } else if (localBall.valid) {
    Serial.println("Only we see the ball");
  } else if (remoteBall.valid) {
    Serial.println("Only the other robot sees the ball");
  }
}
```

### Example 2: Coordinate Between Robots

```cpp
#include "include/subsystems/communication.h"
#include "include/subsystems/localization.h"

void updateStrategy() {
  // Get positions
  RobotPose localPose;
  getRobotPose(localPose);
  
  RobotPose remotePose;
  getRemoteRobotPose(remotePose);
  
  if (localPose.valid && remotePose.valid) {
    // Calculate distance between robots
    float dx = localPose.xMm - remotePose.xMm;
    float dy = localPose.yMm - remotePose.yMm;
    float distance = sqrt(dx*dx + dy*dy);
    
    // If robots are too close, back away
    if (distance < 300) {  // 300mm
      Serial.println("Robots too close - backing away");
      // Add avoidance logic here
    }
  }
}
```

### Example 3: Share Opponent Information

```cpp
#include "include/subsystems/communication.h"
#include "include/subsystems/localization.h"

void updateStrategy() {
  // Get opponents detected by this robot
  OpponentRobot localOpponents[5];
  int localCount = 0;
  getOpponents(localOpponents, 5, localCount);
  
  // Get opponents detected by other robot
  OpponentRobot remoteOpponents[5];
  int remoteCount = 0;
  getRemoteOpponents(remoteOpponents, 5, remoteCount);
  
  // Combined opponent tracking
  Serial.print("Local opponents: ");
  Serial.print(localCount);
  Serial.print(", Remote opponents: ");
  Serial.println(remoteCount);
  
  // Act on opponent positions
  for (int i = 0; i < remoteCount; i++) {
    if (remoteOpponents[i].valid) {
      Serial.print("Other robot detects opponent at X=");
      Serial.print(remoteOpponents[i].xMm);
      Serial.print(", Y=");
      Serial.println(remoteOpponents[i].yMm);
    }
  }
}
```

### Example 4: Check Which Robot You Are

```cpp
#include "include/subsystems/communication.h"

void setup() {
  uint8_t robot = getCurrentRobotNumber();
  Serial.print("I am Robot #");
  Serial.println(robot);
  
  if (robot == 1) {
    Serial.println("I am the ATTACKER");
  } else {
    Serial.println("I am the DEFENDER");
  }
}
```

### Example 5: Monitor Communication Link Health

```cpp
#include "include/subsystems/communication.h"

void loop() {
  // Periodically check communication health
  static unsigned long lastCheckMs = 0;
  if (millis() - lastCheckMs > 5000) {  // Check every 5 seconds
    lastCheckMs = millis();
    printCommunicationStats();
  }
}
```

## Data Freshness

Always check the `timestampMs` field to ensure data is fresh:

```cpp
#include "include/subsystems/communication.h"

void updateStrategy() {
  RobotPose remotePose;
  getRemoteRobotPose(remotePose);
  
  if (remotePose.valid) {
    unsigned long age = millis() - remotePose.timestampMs;
    
    if (age < 100) {
      // Data is fresh (less than 100ms old)
      Serial.println("Fresh pose data");
    } else if (age < 500) {
      // Data is moderately old
      Serial.println("Moderately fresh pose data");
    } else {
      // Data is stale
      Serial.println("Warning: Stale pose data!");
    }
  }
}
```

## Integration Points

### In your main strategy loop:

```cpp
void updateStrategy() {
  // Your existing code...
  
  // NEW: Get remote robot data
  RobotPose remotePose;
  getRemoteRobotPose(remotePose);
  
  FieldBall remoteBall;
  getRemoteFieldBall(remoteBall);
  
  // Decide what to do based on both local and remote data
  // ...
}
```

## Common Pitfalls

### 1. Forgetting to Check Valid Flag

```cpp
// ❌ WRONG - might use invalid data
FieldBall remoteBall;
getRemoteFieldBall(remoteBall);
float distance = remoteBall.distanceCm;  // Could be garbage if not valid

// ✅ CORRECT
FieldBall remoteBall;
getRemoteFieldBall(remoteBall);
if (remoteBall.valid) {
  float distance = remoteBall.distanceCm;
}
```

### 2. Not Checking Data Age

```cpp
// ❌ WRONG - using potentially stale data
RobotPose remotePose;
getRemoteRobotPose(remotePose);
float x = remotePose.xMm;  // From how long ago?

// ✅ CORRECT
RobotPose remotePose;
getRemoteRobotPose(remotePose);
unsigned long age = millis() - remotePose.timestampMs;
if (remotePose.valid && age < 500) {  // Less than 500ms old
  float x = remotePose.xMm;
}
```

### 3. Forgetting about Transmission Latency

The data you receive from the other robot is typically 50-200ms old due to:
- Transmission time
- Deserialization time
- Buffering on the receiving end

Always assume data is slightly stale and factor that into your strategy.

## Transmission Schedule

Data is sent in this pattern (repeating every 100ms):

```
Time 0ms:   Send Pose
Time 50ms:  Send Ball
Time 50ms:  Send Pose
Time 100ms: Send Opponents
Time 100ms: Send Pose
Time 150ms: Send Ball
Time 150ms: Send Pose
```

This means fresh pose data arrives every 50ms, ball every 50ms, opponents every 100ms.

## Button Behavior

### Button 2 (Robot Selection)
- **While not pressed**: Operating as Robot 1 (Attacker)
- **While pressed**: Operating as Robot 2 (Defender)
- **Status**: Printed to serial when changed

Example serial output when button 2 is pressed:
```
Robot number changed to: 2
```

## Serial Debugging

Enable debugging output by adding this to the top of your strategy file:

```cpp
void printCommunicationDebug() {
  static unsigned long lastPrintMs = 0;
  if (millis() - lastPrintMs > 1000) {
    lastPrintMs = millis();
    
    RobotPose remote;
    getRemoteRobotPose(remote);
    
    Serial.print("Remote Pose - Valid: ");
    Serial.print(remote.valid);
    Serial.print(", X: ");
    Serial.print(remote.xMm);
    Serial.print(", Y: ");
    Serial.print(remote.yMm);
    Serial.print(", Age: ");
    Serial.print(millis() - remote.timestampMs);
    Serial.println("ms");
    
    printCommunicationStats();
  }
}
```

Then call `printCommunicationDebug()` in your update loop.
