// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static enum Side {
    SOURCE,
    DEPOT
  }

  public static enum RobotSide {
    LEFT,
    RIGHT
  }

  public static final class IntakePivotConstants {
    public static final int kIntakePivotID = 22;
    public static final double kIntakeGearRatio = 4;
    public static final double kP = 2;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kS = 0.0;
    public static final double kV = 0.8;
    public static final double kA = 0.0;
    public static final double kG = 0.0;
  }

  public static final class IntakeConstants {
    public static final int kIntakeID = 19;
    public static final double kIntakeGearRatio = 2;
  }

  public static final class IndexConstants {
    public static final int kIndexID = 16;
    public static final double kIndexGearRatio = 1.0;
  }

  public static final class FeederConstants {
    public static final int kFeederID = 41;
    public static final double kFeederGearRatio = 1.0;
  }

  public static final class ShooterConstants {
    public static final int LeftShooterLeaderID = 50;
    public static final int LeftShooterFollowerID = 51;
    public static final int RightShooterLeaderID = 52;
    public static final int RightShooterFollowerID = 53;
    public static final double kShooterGearRatio = 1.0;
    public static final double kS = 0.01;
    public static final double kP = 0.0075 / 100 * 4;
    public static final double kD = 0.0075 / 100 * 3;
    public static final double kV = 0.00176;
    /** Fixed launch speed (m/s) for the ball. Only the hood angle varies with distance. */
    public static final double kFixedLaunchVelocityMetersPerSecond = 15.0;
    /** Fixed flywheel RPM when shooting (matches fixed launch velocity). */
    public static final double kFixedShooterRPM = 2360;
  }

  public static final class TurretConstants {
    public static final int kTurretIDLeft = 20;
    public static final int kTurretIDRight = 21;
    public static final int kInitialTurretPosition = 0;
    /**
     * Offset (rad) from turret physical 0° to robot forward. Left turret 0° = +Y (left), so forward
     * = −π/2 (applied as targetYaw − offset). Right turret 0° = −Y (right), so forward = +π/2
     * (applied as −targetYaw + offset).
     */
    public static final double kTurretZeroOffsetRad = Math.PI / 2;
    /** Encoder position units per one full turret revolution [0, 2π] rad. Calibrate with Turret.calibrateEncoderRangeCommand(). */
    public static final double kEncoderUnitsPerRevolution = 10.0;
    public static final double kTurretGearRatio = 1.0;
    public static final double kP = 0.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kS = 3.0;
    public static final double kV = 0.0;
    public static final double kA = 0.0;
    public static final double latency = 0.02;
  }

  public static final class TargetConstants {
    /** Hub position in field coordinates (e.g. red alliance side). Used with {@link #fieldLengthX} for blue mirror. */
    public static final Translation2d hub = new Translation2d(11.915, 4.035);
    public static final double yf = 1.32;
    /** Field length (m) along X for alliance mirror. Blue hub = (fieldLengthX - hub.x, hub.y). */
    public static final double fieldLengthX = 16.54;
  }

  public static final class HoodConstants {
    public static final int kLeftHoodMotorID = 30;
    public static final int kRightHoodMotorID = 31;
    /** Motor rotations per one hood rotation. Sets scale: radians per encoder rotation = 2π/kHoodGearRatio. */
    public static final double kHoodGearRatio = 1.0;
    /** Encoder position (motor rotations) when the left hood is at the reference angle (e.g. facing up). */
    public static final double kLeftEncoderOffsetRotations = 0.0;
    /** Encoder position (motor rotations) when the right hood is at the reference angle (e.g. facing up). */
    public static final double kRightEncoderOffsetRotations = 0.0;
    /** Hood angle (radians) per motor rotation. Derived from gear ratio; one encoder rotation = this many rad. */
    public static final double kEncoderToHoodRadiansPerRotation = (2 * Math.PI) / kHoodGearRatio;
    /** Angle (rad) when the hood is at its reference position. 0 = straight up; angle increases to π (180°) at straight down. */
    public static final double kHoodReferenceAngleRad = 0.0;
    public static final int kCurrentLimitAmps = 20;

    /** Hood angle (rad): 0 = up, π/2 = horizontal, π = down. */
    public static final double kHoodMinAngleRad = 0.0;
    public static final double kHoodMaxAngleRad = Math.PI;
    public static final double kP = 2.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kS = 0.0;
    public static final double kV = 0.8;
    public static final double kA = 0.0;
    public static final double kMotionMagicCruiseVelocity = 1.5;
    public static final double kMotionMagicAcceleration = 15.0;
  }
}
