// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.Shooters;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import static frc.robot.Constants.HoodConstants.*;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.RobotSide;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.SimFiles.TurretSim;
import org.littletonrobotics.junction.Logger;

/**
 * Hood subsystem using a REV Spark MAX + NEO 550. Each hood uses its own internal relative encoder
 * and supports a side-specific zero offset.
 */
public class Hood extends SubsystemBase {
  private final TalonFXS motor;
  private final MotionMagicVoltage hoodRequest = new MotionMagicVoltage(0);
  private final String logPrefix;
  private final double encoderOffsetRotations;
  private double lastSetpointRad = 0;
  private final TurretSim simTurret;

  public Hood(int motorID, RobotSide side, TurretSim simTurret) {
    this.logPrefix = "Hood/" + (side == RobotSide.LEFT ? "Left" : "Right");
    this.encoderOffsetRotations =
        side == RobotSide.LEFT ? kLeftEncoderOffsetRotations : kRightEncoderOffsetRotations;

    this.simTurret = simTurret;

    motor = new TalonFXS(motorID);

    TalonFXSConfiguration cfg =
        new TalonFXSConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(80))
                    .withStatorCurrentLimitEnable(true))
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(RotationsPerSecond.of(5))
                    .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(75))
                    .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(100)))
            .withSlot0(
                new Slot0Configs()
                    .withKP(TurretConstants.kP)
                    .withKI(TurretConstants.kI)
                    .withKD(TurretConstants.kD)
                    .withKS(TurretConstants.kS)
                    .withKV(TurretConstants.kV)
                    .withKA(TurretConstants.kA)
                    .withKG(TurretConstants.kG));

    cfg.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;

    motor.getConfigurator().apply(cfg);

    motor.getConfigurator().setPosition(HoodConstants.kInitialHoodPosition);
  }

  public Command resetEncoderCount() {
    return Commands.run(
        () -> {
          motor.getConfigurator().setPosition(encoderOffsetRotations);
        });
  }

  /**
   * Sets the hood angle setpoint in radians (0 = up, π/2 = horizontal, π = down). Clamped to
   * [kHoodMinAngleRad, kHoodMaxAngleRad].
   */
  public void setAngleRad(double angleRad) {
    double clamped = Math.max(kHoodMinAngleRad, Math.min(kHoodMaxAngleRad, angleRad));
    lastSetpointRad = clamped;
    double setpointRotations =
        (clamped - kHoodReferenceAngleRad) / kEncoderToHoodRadiansPerRotation
            + encoderOffsetRotations;
    motor.setControl(hoodRequest.withPosition(setpointRotations));
  }

  /** Sets the hood angle setpoint in degrees. */
  public void setAngleDegrees(double angleDeg) {
    setAngleRad(Math.toRadians(angleDeg));
  }

  /** Returns the current hood angle in radians (0 = up, π/2 = horizontal, π = down). */
  public double getAngleRad() {
    double positionRotations = motor.getPosition().getValueAsDouble();
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
   * Continuously updates setpoint from ShotCalc for hub tracking. Converts launch angle (0 =
   * horizontal, π/2 = up) to hood angle (0 = up, π/2 = horizontal, π = down): hoodAngle = π/2 -
   * launchPitch.
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
              double pos = motor.getPosition().getValueAsDouble();
              Logger.recordOutput(logPrefix + "/CalibrationEncoderPosition", pos);
            })
        .withName("HoodCalibrateOffset");
  }

  @Override
  public void periodic() {
    double angleRad = getAngleRad();
    double rawEncoderRotations = motor.getPosition().getValueAsDouble();
    Logger.recordOutput(logPrefix + "/AngleRad", angleRad);
    Logger.recordOutput(logPrefix + "/AngleDeg", Math.toDegrees(angleRad));
    Logger.recordOutput(logPrefix + "/SetpointRad", lastSetpointRad);
    Logger.recordOutput(logPrefix + "/EncoderRotations", rawEncoderRotations);
    Logger.recordOutput(logPrefix + "/CurrentAmps", motor.getStatorCurrent().getValueAsDouble());
    Logger.recordOutput(
        logPrefix + "/Voltage",
        motor.getMotorVoltage().getValueAsDouble() * motor.getSupplyVoltage().getValueAsDouble());
    // Hood only moves when a command runs (e.g. trackHub() via L2)
  }
}
