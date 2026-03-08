# Shooting and Ballistics

This document describes the math and behavior used to aim the turret and hood at the hub (or any target), and how the code keeps the shot consistent with the robot’s pose and the target’s position.

---

## Overview

- **ShotCalc** (`subsystems/Shooters/ShotCalc.java`) does all aim and ballistic math. It uses:
  - Robot pose from **Drive**
  - Target position in field coordinates (default: **hub** from `TargetConstants`)
  - Vertical offset to target **yf** (hub height minus shooter height)
- **Turret** rotates to the correct **yaw** (robot-relative angle to the target) and uses **shortest-path** so it can wrap back toward zero past 360°.
- **Hood** sets its angle from **pitch** (launch angle) so the ball reaches the target height at the current horizontal range.

All calculations use the **pose of the hub** (or whatever target is set via `ShotCalc.setTarget()`). Call `ShotCalc.updateState()` once per cycle before using range, yaw, or pitch.

---

## Coordinate system

- **Field**: WPILib convention. X forward (e.g. toward red alliance wall), Y left. Units: meters.
- **Robot pose**: `(x, y, rotation)`. `rotation = 0` means the robot faces +X.
- **Target (hub)**: Stored as a `Translation2d` in field coordinates. Default: `TargetConstants.hub` (e.g. 11.915 m, 4.035 m). Can be changed with `ShotCalc.setTarget(...)`.
- **yf**: Vertical distance from shooter exit to target (meters). `TargetConstants.yf` (e.g. 1.32 m). Used in all ballistic formulas.

Turret position on the robot is modeled by a 2D offset from the robot pose; that offset is used to compute the “turret position” point for range and yaw.

---

## Horizontal range (xf)

- **Definition**: Horizontal distance from the turret position (robot pose + turret offset) to the target.
- **Formula**:
  `xf = sqrt((target.x - turretX)² + (target.y - turretY)²)`
- **Usage**:
  - Input to `getPitchForDistance(xf)` for the hood.
  - Input to `getVelocity(xf)` for shooter speed.
  - Updated in `updateState()` using the current target (hub or custom).

So both **hood** and **shooter** use the same horizontal range to the **hub pose** (or current target).

---

## Turret yaw (aim angle)

- **Goal**: Turret angle in the robot frame so the shooter points at the target.
- **Formula in ShotCalc**:
  - Turret position in field: `turretPose = robotPose + turretOffset` (2D).
  - `deltax = target.x - turretPose.x`, `deltay = target.y - turretPose.y`.
  - Field angle to target: `initTheta = π - atan2(deltay, -deltax)`.
  - Robot-relative yaw: `theta = initTheta - robotPose.getRotation().getRadians()`.
  - Returned in **[0, 2π)** via `mod(theta)`.

**Important**: `getYaw(Pose2d)` uses the instance field **`target`** (set by `setTarget()`), not the constant `TargetConstants.hub` directly. So if you call `setTarget(...)`, both range and yaw use that target.

### Shortest path (wrap past 360°)

- The turret has a finite range (e.g. one revolution, 0–2π rad). We want the setpoint that still points at the target but minimizes rotation (so it can “rotate back to zero” instead of going the long way).
- **getYawShortestPath(robotPose, currentTurretRad)**:
  - Computes `targetYaw = getYaw(robotPose)` in [0, 2π).
  - Returns the equivalent angle in [0, 2π) that is closest to `currentTurretRad` (i.e. minimizes |setpoint − current| on the circle).
- **getYawEquivalentClosestTo(targetAngleRad, currentTurretRad)**:
  - Same idea for an arbitrary target angle (e.g. for the right turret, where the physical angle is `-yaw`). Used so the turret takes the short path.

**Turret behavior**: In `periodic()`, the turret reads the current encoder angle, gets the target yaw (with sign for left/right), then uses `getYawEquivalentClosestTo` to get the setpoint and calls `run(setpointRad)`. So the turret actually moves every cycle and can wrap past 360° via the shortest path.

---

## Hood pitch (launch angle)

- **Goal**: Launch angle (radians) so the ball reaches the target height **yf** at horizontal range **xf**.
- **Ballistic model**:
  - Projectile: `x(t) = v cos(θ) t`, `y(t) = v sin(θ) t − ½ g t²`.
  - We want `(xf, yf)` at some time. Eliminating `t`:
    `yf = xf tan(θ) − g xf² / (2 v² cos²(θ))`.
  - Solving for θ for a given **v** and **(xf, yf)** gives a quadratic in `tan(θ)`. The code uses the same form as TurretSim:
    `A = a·xf²/(2v²)` with `a = −g`, `B = xf`, `C = A − yf`, then
    `θ = atan((−B − sqrt(B² − 4 A C)) / (2 A))`.
- **getPitchForDistance(xf)**:
  - First gets launch speed with `getVelocityForDistance(xf)` (see below).
  - Then solves for pitch with the quadratic above.
  - If the discriminant is negative, returns a fallback (e.g. 60°).

So the **hood** adjusts correctly relative to the robot: it uses the same **xf** (distance to hub) and **yf** (height to hub) so the ball goes to the **position of the hub**.

---

## Launch speed (velocity)

Two formulas are used:

1. **getVelocity(xf)** (fixed pitch, e.g. 60°):
   From projectile motion,
   `v² = g·xf² / (2 cos²(θ)·(xf tan(θ) − yf))`.
   Used when pitch is fixed.

2. **getVelocityForDistance(xf)** (used by getPitchForDistance):
   Empirical formula aligned with TurretSim:
   `v = sqrt(g·yf + g·sqrt(xf² + yf²)) + 0.5`.
   So pitch and velocity for the hood are consistent with this model.

---

## Summary of fixes and behavior

| Item | What was wrong / unclear | What was done |
|------|---------------------------|---------------|
| **getYaw target** | Used `Constants.TargetConstants.hub` instead of the instance `target`. So `setTarget()` had no effect on yaw. | `getYaw` now uses the instance field `target` so hub pose (or any set target) is used for both range and yaw. |
| **Turret not moving** | `periodic()` called `setTurretPos(targetYaw)`, which only *returns* a Command and never ran the motor. | `periodic()` now computes the setpoint and calls `this.run(setpointRad)` so the turret is driven every cycle. |
| **Turret long way** | No shortest-path logic; turret could spin the long way past 360°. | Added `getYawShortestPath` and `getYawEquivalentClosestTo`; turret uses them so the setpoint is the equivalent angle closest to the current position. |
| **Right turret** | Same yaw used for both sides. | Target angle for the right turret is `mod(-getYaw(pose))`; shortest path is applied to that so each side points at the hub and takes the short path. |
| **Hood** | Already used `getPitchForDistance(shotCalc.getXf())` and `updateState()`. | No formula change. Hood angle is correct relative to robot and hub position; docs added. |

---

## File reference

- **ShotCalc**: `src/main/java/frc/robot/subsystems/Shooters/ShotCalc.java` — all ballistic and yaw math; uses `target` and `yf`; `updateState()` must be called before using xf/yaw/pitch.
- **Turret**: `src/main/java/frc/robot/subsystems/Shooters/Turret.java` — in `periodic()`, updates state, gets shortest-path setpoint, runs turret with `run(setpointRad)`.
- **Hood**: `src/main/java/frc/robot/subsystems/Shooters/Hood.java` — default command `trackHub()` calls `updateState()` and `setAngleRad(getPitchForDistance(getXf()))`.
- **Target constants**: `Constants.TargetConstants.hub` (field position), `TargetConstants.yf` (height to target).
