// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
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

  public static final class IntakeConstants {
    public static final int kIntakeLeaderID = 21;
    public static final int kIntakeFollowerID = 22;
    public static final double kIntakeGearRatio = 2.0;
  }

  public static final class IntakePivotConstants {
    public static final int kPivotLeaderID = 10;
    public static final int kPivotFollowerID = 11;
    public static final double kIntakeGearRatio = 50;
    public static final double kInitialPivotPosition = 0.62;
    public static final double kP = 6.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kS = 0.0;
    public static final double kV = 0.0;
    public static final double kA = 0.0;
    public static final double kG = 2.0;
  }

  public static final class IndexConstants {
    public static final int kIndexLeaderID = 30;
    public static final int kIndexFollowerID = 31;
    public static final double kIntexGearRatio = 1.0;
  }

  public static final class ShooterConstants {
    public static final int kShooterLeaderID = 50;
    public static final int kShooterFollowerID = 51;
    public static final double kShooterGearRatio = 1.0;
    public static final double kP = 0.4;
    public static final double kI = 0.0;
    public static final double kD = 0.0165;
    public static final double kS = 0.0;
    public static final double kV = 0.15;
    public static final double kA = 0.0;
    public static final double kG = 0.0;
  }

  public static final class HoodConstants {
    public static final double kInitialHoodEncoderPosition = 0.67905;
    public static final double kHoodGearRatio = 16.2;
    public static final double kInitialHoodAnglePosition = kInitialHoodEncoderPosition / kHoodGearRatio * 360;
    public static final double kMaxHoodAnglePosition = 55;
    public static final int kHoodMotorID = 40;
    public static final int kHoodEncoderID = 0;
    public static final double kP = 35.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kS = 0.3;
    public static final double kV = 0.8;
    public static final double kA = 0.032;
    public static final double kG = 0.0;
  }

  public static final class TargetConstants {
    public static final Translation2d hub =
        new Translation2d(
            DriverStation.getAlliance().get() == DriverStation.Alliance.Red
                ? 11.915
                : 16.54 - 11.915,
            4.035);
    public static final double yf = 1.32;
    public static final Transform2d shooterOffset = new Transform2d(-0.17, 0, new Rotation2d());
  }
}
