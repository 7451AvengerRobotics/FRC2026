# Robot Architecture

## Overview

The robot software uses the **WPILib command-based framework** with **AdvantageKit** logging. The entry point is `Robot.java`, which extends `LoggedRobot` and runs the `CommandScheduler` in `robotPeriodic()`. All structure—subsystems, commands, and button bindings—is declared in `RobotContainer.java`, which is constructed once at robot startup.

- **Modes**: The code supports `REAL` (roboRIO), `SIM` (physics sim), and `REPLAY` (log replay) via `Constants.currentMode`.
- **Default drive**: The drivetrain’s default command is field-relative joystick drive; other commands are triggered by controller buttons or the autonomous chooser.
- **Autonomous**: A `LoggedDashboardChooser<Command>` provides the autonomous command; PathPlanner’s `AutoBuilder` is also registered for path-based autos.

---

## Subsystems

### Drive (`subsystems.drive.Drive`)

- **Purpose**: Swerve drivetrain: pose estimation, velocity control, and PathPlanner integration.
- **Hardware**: Pigeon 2 gyro (`GyroIOPigeon2`), four swerve modules (FL, FR, BL, BR). On real robot: `ModuleIOTalonFX` (TalonFX drive/turn, CANcoder) from `TunerConstants`; in sim: `ModuleIOSim`; in replay: no-op `ModuleIO`/`GyroIO`.
- **Key methods**:
  - `runVelocity(ChassisSpeeds)` — main drive output (discretized, desaturated).
  - `getPose()` / `setPose(Pose2d)` — odometry.
  - `addVisionMeasurement(...)` — consumed by Vision for pose correction.
  - `driveToPose(Pose2d)` — holonomic PID to a pose.
  - `followPPPathCommand(String pathName)` — load PathPlanner path and follow it.
  - `driveOverBump(Side)` — drive to initial pose then follow Depot/Source bump path.
  - SysId: `sysIdQuasistatic`, `sysIdDynamic`; characterization: `runCharacterization(double)`.

### Vision (`subsystems.vision.Vision`)

- **Purpose**: Multi-camera pose from AprilTags and simple target angles for logging/servoing.
- **Hardware**: One or more vision backends. Real: `VisionIOPhotonVision` (PhotonVision camera, e.g. `VisionConstants.camera0Name` "BackLeft"); Sim: `VisionIOPhotonVisionSim`; Replay: no-op `VisionIO`.
- **Key methods**:
  - Pose observations are pushed to `Drive` via the `VisionConsumer` (`drive::addVisionMeasurement`) with timestamp and std devs.
  - `getTargetX(int cameraIndex)` — horizontal angle to best target (e.g. for servoing).

### Turret (`subsystems.Shooters.Turret`)

- **Purpose**: Single turret axis (yaw) for aiming at the hub (or target set on ShotCalc). Uses shortest-path so it can wrap back toward zero past 360°.
- **Hardware**: TalonFXS at `TurretConstants.kTurretID` (20), MotionMagic position control. Uses `ShotCalc` for target yaw and `Drive` for robot pose.
- **Key methods**:
  - `run(double angleRad)` — set turret position in radians (mapped to encoder 0–10 via `angleToEncoder`).
  - `setTurretPos(double angle)` — command that continuously runs `run(angle)`.
  - `stopTurret()` — command that runs turret at 0.
  - In `periodic()`: `shotCalc.updateState()`, then computes setpoint via `getYawEquivalentClosestTo` (shortest path), then `this.run(setpointRad)` so the turret is driven every cycle.
- **See also**: [Shooting and ballistics](shooting_ballistics.md) for yaw formula and shortest-path behavior.

### TurretSim (`subsystems.SimFiles.TurretSim`)

- **Purpose**: Simulation-side turret: ball trajectory (FuelSim), yaw/velocity/pitch logging, and a one-shot “shoot” that spawns a simulated projectile.
- **Hardware**: None; uses `Drive` pose and `ShotCalc` (and optionally a `Transform3d` offset).
- **Key methods**:
  - `shootBallCommand()` — one-shot command that calls `shootBall()` (adds a `FuelSim` to the list).
  - `calcYaw()`, `calcVelocity()`, `calcPitch()` — used for logging and sim shot.

### Shooter (`subsystems.Shooters.Shooter`)

- **Purpose**: Dual-motor flywheel (leader + follower) with optional closed-loop velocity; shot velocity from `ShotCalc`.
- **Hardware**: Two SparkFlex (REV): leader and follower at IDs from `ShooterConstants` (left: 50/51, right: 52/53). Velocity control via MAXMotion; PD + kS/kV feedforward.
- **Key methods**:
  - `run(double power)` — duty cycle.
  - `setVel(double rpm)` — closed-loop velocity (MAXMotion).
  - `runShooter()` — command that runs at fixed 2360 RPM.
  - `flywheelVel()` — computes RPM from `shotCalc.getVelocity(shotCalc.getXf())` and a quadratic conversion.
  - `stopShooter()` — command to stop.

### ShotCalc (`subsystems.Shooters.ShotCalc`)

- **Purpose**: Ballistics for a fixed target (hub): range, required velocity, yaw, and (optionally) moving-shot corrections.
- **Hardware**: None; uses `Drive` for robot pose and `Constants.TargetConstants.hub` (e.g. 11.915 m, 4.035 m) and `yf` (1.32 m).
- **Key methods**:
  - `updateState()` — updates turret pose in 2D, field-relative chassis speeds, and horizontal range `xf`.
  - `getVelocity(double xf)` — required launch speed for fixed pitch (e.g. 60°).
  - `getYaw(Pose2d)` — aim angle to hub in turret frame.
  - `getMovingVelocity`, `getMovingYaw` — velocity and yaw adjusted for robot motion.

### Intake (`subsystems.Intake.Intake`)

- **Purpose**: Run game pieces into the robot.
- **Hardware**: TalonFX at `IntakeConstants.kIntakeID` (19), duty-cycle output.
- **Key methods**:
  - `setIntakePower(double)`, `runIntake(double power)`, `stopIntake()`.
  - `propIntake()` — stall detection (velocity and current thresholds); logged to NetworkTables "Intake".

### IntakePivot (`subsystems.Intake.IntakePivot`)

- **Purpose**: Pivot the intake to stow or deploy.
- **Hardware**: TalonFX at `IntakePivotConstants.kIntakePivotID` (22), MotionMagic position.
- **Key methods**:
  - `pivotIntake(double rotations)`, `toPosition(double rotations)` (command until `nearSetpoint`), `nearSetpoint(double)`.
  - Positions: 0 ≈ stow; 0.27 used for intake; 2.8 used for “deploy” in bindings.

### Index (`subsystems.Index`)

- **Purpose**: Move game pieces between intake and shooter/feeder.
- **Hardware**: TalonFX at `IndexConstants.kIndexID` (16), duty-cycle.
- **Key methods**: `run(double speed)`, `runIndex(double power)`, `stopIndex()`, `reverseIndex()` (via SuperStructure).

### Feeder (`subsystems.Feeder`)

- **Purpose**: Feed game pieces into the shooter.
- **Hardware**: TalonFX at `FeederConstants.kFeederID` (41), duty-cycle.
- **Key methods**: `run(double speed)`, `runFeeder(double power)`, `stopFeeder()`.

### SuperStructure (coordinator)

- **Purpose**: Composes intake, index, feeder, shooters, turrets, and intake pivot into high-level commands.
- **Hardware**: None; holds references to the above subsystems.
- **Key methods**:
  - Intake/index/feeder: `startIntake()`, `stopIntake()`, `stopIndex()`, `stopFeeder()`, `reverseIndex()`, `soleIntake()`, `soleIndex()`, `soleFeeder()`.
  - Shooting: `leftShoot()`, `rightShoot()`, `runShooters()`, `stopShooters()`.
  - Combined: `masterCommand()` (intake + index + feeder + both shooters), `weirdMasterCommand()` (intake + index opposite direction + feeder), `stopMasterCommand()`, plus shooterless/intakeless variants.
  - Pivot: `deployPivot()`, `stowPivot()`.
  - Turret: `stopTurret()`.

---

## Commands

- **Drive**
  - **Default**: `DriveCommands.joystickDrive(drive, leftY, leftX, rightX)` — field-relative, deadbanded, squared inputs; alliance flip for Red.
  - **DriveCommands.joystickDriveAtAngle** — same linear input, rotation from a PID to a supplied angle (e.g. vision).
  - **DriveCommands.feedforwardCharacterization(drive)** / **wheelRadiusCharacterization(drive)** — characterization only.
- **SuperStructure**
  - **masterCommand** — intake (-0.5), index (-0.9), feeder (-0.9), both shooters.
  - **weirdMasterCommand** — intake (-0.75), index (+0.6), feeder (-0.9); no shooters.
  - **stopMasterCommand** — stop intake, index, feeder, shooters.
  - **deployPivot** — pivot to 2.8; **stowPivot** — pivot to 0.
  - **startIntake** — pivot to 0.27, intake -0.5, index -0.1.
  - **leftShoot** / **rightShoot** — run left or right shooter at fixed RPM (e.g. 2360).
- **TurretSim**
  - **simTurretLeft.shootBallCommand()** — one-shot sim “shoot” (spawns FuelSim).
- **Drive**
  - **driveToPose(Pose2d)** — holonomic PID until at pose.
  - **followPPPathCommand(pathName)** — PathPlanner path follow.
  - **driveOverBump(Side)** — initial pose then path "DepotSideBump" or "SourceSideBump".

---

## Operator Controls

- **Controller**: `CommandPS5Controller` on port 0.
- **Bindings** (from `RobotContainer.configureButtonBindings()`):
  - **Default command**: Field-relative drive with left stick (Y, X) and right X for rotation.
  - **Triangle**: `superStructure.weirdMasterCommand()`.
  - **Circle**: `superStructure.stopMasterCommand()`.
  - **Cross**: `superStructure.masterCommand()`.
  - **Square**: `superStructure.deployPivot()`.
  - **L1**: `simTurretLeft.shootBallCommand()` (sim shoot).
  - **R1**: `superStructure.rightShoot()` (run right shooter).
  - **PS**: `superStructure.stopShooters()`.

Commented out in code: X (drive stop-with-X), L2 (start intake), L1/R1 as turret shoot or left/right shoot.

---

## Drivetrain System

- **Type**: Swerve drive with four modules (FL, FR, BL, BR). Kinematics and pose estimator use module positions from `TunerConstants` (locations, wheel radius, gear ratios). Odometry runs in `Drive.periodic()` using timestamps and positions from a **PhoenixOdometryThread** (high-rate Phoenix 6 signals).
- **Control**: `runVelocity(ChassisSpeeds)` converts to module states, discretizes and desaturates, then each `Module.runSetpoint(SwerveModuleState)` sets drive velocity and turn position (with cosine scaling). Gyro is used when connected; otherwise rotation is inferred from kinematics.
- **Commands**: Default is `joystickDrive` (field-relative). `driveToPose` uses `HolonomicDriveWithPIDController` (x, y, theta PID) and runs until `atReference()`. PathPlanner paths use `AutoBuilder` and `followPPPathCommand` (resets odom to path start then follows path).

---

## Shooting System

- **Flow**: Game pieces go Intake → Index → Feeder → Shooter(s). Left and right sides each have a **Shooter** (flywheel), **Turret** (yaw), and **Hood** (pitch). **ShotCalc** uses robot pose and the current target (default hub) to compute range (`xf`), required velocity, yaw, and pitch.
- **Shooter**: SparkFlex leader/follower; `runShooter()` uses a fixed 2360 RPM; `flywheelVel()` can compute RPM from `ShotCalc` for dynamic shots.
- **Turret**: In `periodic()`, `ShotCalc.updateState()` is called, then the setpoint is computed with shortest-path (`getYawEquivalentClosestTo`) so the turret can wrap back toward zero past 360°, then `this.run(setpointRad)` drives the motor. Uses `target` from ShotCalc (hub pose or `setTarget()`). Left/right sign applied by `RobotSide`.
- **Hood**: Default command `trackHub()` runs `updateState()` and `setAngleRad(getPitchForDistance(getXf()))` so the hood tracks the required launch angle for the current distance to the hub.
- **Sim**: `TurretSim` logs yaw/velocity/pitch and provides `shootBallCommand()` to simulate a shot (adds a `FuelSim` with velocity/pitch/yaw from `ShotCalc`).
- **Target**: `ShotCalc.setTarget()` (default `TargetConstants.hub`) and `TargetConstants.yf`; both yaw and pitch use the hub pose. See [Shooting and ballistics](shooting_ballistics.md) for formulas and behavior.

---

## Vision System

- **Stack**: **Vision** subsystem holds one or more `VisionIO` implementations. On real robot: **PhotonVision** (`VisionIOPhotonVision`) with camera name and `robotToCamera` from `VisionConstants` (e.g. "BackLeft", transform in inches/degrees). Sim: `VisionIOPhotonVisionSim` (uses `drive::getPose` for simulated observations). Replay: no-op IO.
- **Pose**: Each camera produces pose observations (single-tag or multitag). Vision filters by ambiguity, Z, and field bounds, then computes std devs (distance, tag count, MegaTag2 factors) and calls `consumer.accept(pose2d, timestamp, stdDevs)` — the consumer is `drive::addVisionMeasurement`, so vision corrects the drive pose estimator.
- **Target angle**: `getTargetX(cameraIndex)` returns the latest target’s yaw (e.g. for simple vision servoing); not currently wired to a command in the scanned bindings.

---

## Autonomous System

- **Selection**: `LoggedDashboardChooser<Command>` "Auto Choices". PathPlanner’s `AutoBuilder.buildAutoChooser()` is the first source; then `configureAutos()` adds:
  - Drive characterizations: wheel radius, feedforward, SysId quasistatic/dynamic (forward/reverse).
  - Three routines from **AutoRoutines**: **Depot Side Auto**, **Middle Auto**, **Source Side Auto**.
- **AutoRoutines** (all share the same pattern):
  1. `drive.driveToPose(...)` to an initial pose (depot: (3.5, 6.5); middle: (3.5, 4); source: (3.5, 1.5)).
  2. In parallel: `superStructure.deployPivot()` and `drive.followPPPathCommand("...StartToSource")` (path name varies by routine).
  3. Wait 3 seconds.
  4. In parallel: `superStructure.startIntake()` and `drive.followPPPathCommand("SourceToDepot")`.
  5. `drive.followPPPathCommand("DepotToShoot")`.
- PathPlanner uses `AutoBuilder` (pose, chassis speeds, runVelocity, config, alliance flip) and **LocalADStarAK** for pathfinding. Path files (e.g. "DepotSideStartToSource", "SourceToDepot", "DepotToShoot", "DepotSideBump", "SourceSideBump") must exist in the PathPlanner project.

---

## Data Flow

- **Sensors → Subsystems**: Gyro and module IO are read in `Drive.periodic()` (with odometry lock); Vision IO is read in `Vision.periodic()`. Phoenix 6 devices are also sampled in **PhoenixOdometryThread** for high-rate odometry timestamps.
- **Vision → Drive**: Vision calls `drive.addVisionMeasurement(pose2d, timestamp, stdDevs)` for accepted observations; the pose estimator fuses these with wheel and gyro.
- **Drive → Shooting**: `Drive.getPose()` and `getRobotRelativeSpeeds()` are used by `ShotCalc` and turrets (and TurretSim) for aim and velocity.
- **Commands → Subsystems**: Buttons schedule commands that call subsystem methods (e.g. `runShooter()`, `runIntake()`, `runVelocity()`). SuperStructure commands run multiple subsystems in parallel.
- **Logging**: AdvantageKit `Logger` and `@AutoLogOutput` / `processInputs` are used for inputs and key outputs (e.g. odometry, swerve states, vision, turret/shooter state).

---

## Key Classes

| Class | Role |
|------|------|
| **Robot** | LoggedRobot; starts AdvantageKit, creates RobotContainer, runs CommandScheduler in robotPeriodic; schedules chosen auto in autonomousInit; cancels auto in teleopInit. |
| **RobotContainer** | Builds Drive/Vision (mode-dependent IO), Turrets, Shooters, ShotCalcs, SuperStructure, controller; configures default command and button bindings; builds auto chooser and adds characterization + AutoRoutines. |
| **Constants** | `currentMode` (REAL/SIM/REPLAY), `Side` (SOURCE/DEPOT), `RobotSide` (LEFT/RIGHT), and nested constant classes (IntakePivot, Intake, Index, Feeder, Shooter, Turret, TargetConstants). |
| **Drive** | Swerve kinematics, pose estimator, velocity control, PathPlanner AutoBuilder, holonomic drive-to-pose, bump paths, SysId. |
| **Module** | Single swerve module: IO update, setpoint/characterization/stop, odometry position/timestamps. |
| **ModuleIO / ModuleIOTalonFX / ModuleIOSim** | Abstract module IO; real = TalonFX + CANcoder; sim = physics. |
| **GyroIO / GyroIOPigeon2** | Gyro inputs; Pigeon 2 on real. |
| **Vision** | Multi-camera VisionIO; filters observations, sends pose to Drive; exposes getTargetX. |
| **VisionIOPhotonVision / VisionIOPhotonVisionSim** | PhotonVision and sim vision IO. |
| **VisionConstants** | AprilTag layout, camera names, robot-to-camera transforms, filtering and std dev parameters. |
| **HolonomicDriveWithPIDController** | X, Y, and rotation PID for drive-to-pose; outputs field-relative ChassisSpeeds. |
| **SuperStructure** | Composes intake, index, feeder, shooters, turrets, pivot into single-command APIs. |
| **Shooter** | SparkFlex flywheel; velocity or duty cycle; optional ShotCalc-based RPM. |
| **Turret** | TalonFXS yaw; MotionMagic; uses ShotCalc + Drive; shortest-path in periodic. |
| **Hood** | Spark MAX + NEO 550 (pitch); internal relative encoder with side-specific offsets; trackHub() default; uses ShotCalc.getPitchForDistance(xf). |
| **TurretSim** | Sim turret: ShotCalc, FuelSim projectiles, shootBallCommand. |
| **ShotCalc** | Target-relative ballistics: xf, velocity, yaw (and shortest-path), pitch; uses target pose. |
| **Intake / IntakePivot / Index / Feeder** | TalonFX (and pivot MotionMagic); duty-cycle run/stop and pivot positions. |
| **DriveCommands** | joystickDrive, joystickDriveAtAngle, feedforwardCharacterization, wheelRadiusCharacterization. |
| **AutoRoutines** | depotSideAuto, middleAuto, sourceSideAuto (driveToPose → path to source → wait → intake + path to depot → path to shoot). |
| **TunerConstants** | Generated swerve config: module IDs, positions, gear ratios, wheel radius, Pigeon ID, CAN bus, SysId/characterization constants. |
| **PhoenixOdometryThread** | High-rate sampling of Phoenix 6 signals for odometry timestamps. |
| **LocalADStarAK** | Pathfinder implementation for PathPlanner (e.g. dynamic pathfinding). |

---

*This document is derived from the current codebase only. Update `Constants.TargetConstants` and game-specific logic using the 2026 Game Manual as needed.*
