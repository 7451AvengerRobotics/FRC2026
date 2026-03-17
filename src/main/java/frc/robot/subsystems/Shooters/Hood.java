// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.Shooters;

import static frc.robot.Constants.HoodConstants.*;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotSide;
import frc.robot.subsystems.SimFiles.TurretSim;

import org.littletonrobotics.junction.Logger;

/**
 * Hood subsystem using a REV Spark MAX + NEO 550. Each hood uses its own internal relative encoder
 * and supports a side-specific zero offset.
 */
public class Hood extends SubsystemBase {
  private final SparkMax motor;
  private final SparkClosedLoopController closedLoopController;
  private final String logPrefix;
  private final double encoderOffsetRotations;
  private double lastSetpointRad = 0;
  private final TurretSim simTurret;

  public Hood(int motorID, RobotSide side, TurretSim simTurret) {
    this.logPrefix = "Hood/" + (side == RobotSide.LEFT ? "Left" : "Right");
    this.encoderOffsetRotations =
        side == RobotSide.LEFT ? kLeftEncoderOffsetRotations : kRightEncoderOffsetRotations;

    this.simTurret = simTurret;

    motor = new SparkMax(motorID, MotorType.kBrushless);
    closedLoopController = motor.getClosedLoopController();

    SparkMaxConfig motorConfig = new SparkMaxConfig();
    motorConfig.idleMode(IdleMode.kBrake);
    motorConfig.smartCurrentLimit(kCurrentLimitAmps);

    // Keep position in raw motor rotations, but expose velocity as rotations/sec so these constants
    // remain in the same units as the old hood tuning.
    motorConfig.encoder.positionConversionFactor(1.0).velocityConversionFactor(1.0 / 60.0);
    motorConfig.closedLoop.p(kP).i(kI).d(kD);
    motorConfig.closedLoop.feedForward.kS(kS).kV(kV);
    motorConfig.closedLoop.maxMotion.cruiseVelocity(kMotionMagicCruiseVelocity);
    motorConfig.closedLoop.maxMotion.maxAcceleration(kMotionMagicAcceleration);
    motorConfig.closedLoop.maxMotion.allowedProfileError(
        Math.toRadians(1.0) / kEncoderToHoodRadiansPerRotation);

    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Manual-zero workflow: place the hood at its known reference angle before enabling, then seed
    // the relative encoder to that side's offset so angle math starts from the correct reference.
    motor.getEncoder().setPosition(encoderOffsetRotations);
  }

  /** Sets the hood angle setpoint in radians (0 = up, π/2 = horizontal, π = down). Clamped to [kHoodMinAngleRad, kHoodMaxAngleRad]. */
  public void setAngleRad(double angleRad) {
    double clamped = Math.max(kHoodMinAngleRad, Math.min(kHoodMaxAngleRad, angleRad));
    lastSetpointRad = clamped;
    double setpointRotations =
        (clamped - kHoodReferenceAngleRad) / kEncoderToHoodRadiansPerRotation + encoderOffsetRotations;
    closedLoopController.setSetpoint(setpointRotations, ControlType.kMAXMotionPositionControl);
  }

  /** Sets the hood angle setpoint in degrees. */
  public void setAngleDegrees(double angleDeg) {
    setAngleRad(Math.toRadians(angleDeg));
  }

  /** Returns the current hood angle in radians (0 = up, π/2 = horizontal, π = down). */
  public double getAngleRad() {
    double positionRotations = motor.getEncoder().getPosition();
    return (positionRotations - encoderOffsetRotations) * kEncoderToHoodRadiansPerRotation
        + kHoodReferenceAngleRad;
  }

  /** Returns true if current angle is within tolerance of the last setpoint. */
  public boolean atSetpoint(double toleranceRad) {
    return Math.abs(getAngleRad() - lastSetpointRad) < toleranceRad;
  }

  public Command toAngleRad(double angleRad) {
    return run(() -> setAngleRad(angleRad));
  }

  public Command toAngleDegrees(double angleDeg) {
    return run(() -> setAngleDegrees(angleDeg));
  }

  /**
   * Continuously updates setpoint from ShotCalc for hub tracking. Converts launch angle (0 = horizontal,
   * π/2 = up) to hood angle (0 = up, π/2 = horizontal, π = down): hoodAngle = π/2 - launchPitch.
   */
  public Command trackHub() {
    return run(
        () -> {
          double launchPitchRad = simTurret.getRequiredPitch();
          setAngleRad(Math.PI / 2 - launchPitchRad);
        });
  }

  /** Holds current position. */
  public Command stopHood() {
    return run(() -> setAngleRad(getAngleRad()));
  }

  /**
   * Logs the current raw encoder position (motor rotations) for offset calibration. Place this hood
   * at straight up (0° in hood convention), run this command once, then set {@code
   * HoodConstants.kLeftEncoderOffsetRotations} or {@code kRightEncoderOffsetRotations} to the
   * logged value {@code Hood/Left/CalibrationEncoderPosition} or {@code Hood/Right/...}.
   */
  public Command logEncoderPositionForOffsetCalibration() {
    return runOnce(
            () -> {
              double pos = motor.getEncoder().getPosition();
              Logger.recordOutput(logPrefix + "/CalibrationEncoderPosition", pos);
            })
        .withName("HoodCalibrateOffset");
  }

  @Override
  public void periodic() {
    double angleRad = getAngleRad();
    double rawEncoderRotations = motor.getEncoder().getPosition();
    Logger.recordOutput(logPrefix + "/AngleRad", angleRad);
    Logger.recordOutput(logPrefix + "/AngleDeg", Math.toDegrees(angleRad));
    Logger.recordOutput(logPrefix + "/SetpointRad", lastSetpointRad);
    Logger.recordOutput(logPrefix + "/EncoderRotations", rawEncoderRotations);
    Logger.recordOutput(logPrefix + "/CurrentAmps", motor.getOutputCurrent());
    Logger.recordOutput(logPrefix + "/Voltage", motor.getAppliedOutput() * motor.getBusVoltage());
    // Hood only moves when a command runs (e.g. trackHub() via L2)
  }
}