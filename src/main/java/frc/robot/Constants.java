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

  public static final class IntakePivotConstants {
    public static final int kIntakePivotID = 22;
    public static final double kIntakeGearRatio = 1.0; // TODO: Set gear ratio
    public static final double kP = 0.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kS = 0.0;
    public static final double kV = 0.0;
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
    public static final double kS = 0;
    public static final double kP = 5;
    public static final double kV = 0;
  }

  public static final class TurretConstants {
    public static final int kTurretID = 47;
    public static final int kInitialTurretPosition = 0;
    public static final double kTurretGearRatio = 1.0;
    public static final double kP = 0.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kS = 0.0;
    public static final double kV = 0.0;
    public static final double kA = 0.0;
    public static final double latency = 0.02;
  }

  public static final class TargetConstants {
    public static final Translation2d hub = new Translation2d(11.915, 4.035);
  }
}
