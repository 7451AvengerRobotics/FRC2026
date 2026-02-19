// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

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
    public static final int kIntakePivotID = 49;
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
    public static final int kIntakeID = 48;
    public static final double kIntakeGearRatio = 1.0;
  }

  public static final class IndexConstants {
    public static final int kIntakeID = 47;
    public static final double kIndexGearRatio = 1.0;
  }

  public static final class TurretConstants {
    public static final int kTurretID = 47;
    public static final double kTurretGearRatio = 1.0;
    public static final double kP = 0.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kS = 0.0;
    public static final double kV = 0.0;
    public static final double kA = 0.0;
    public static final double latency = 0.02;
  }
}
