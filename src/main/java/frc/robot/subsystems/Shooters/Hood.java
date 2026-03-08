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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotSide;
import org.littletonrobotics.junction.Logger;

/**
 * Hood subsystem using a CTRE Minion motor with a Phoenix (TalonFXS) controller. Encoder feedback
 * comes from the controller (rotor/integrated); no external CANcoder.
 */
public class Hood extends SubsystemBase {
  private final TalonFXS motor;
  private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0);
  private final ShotCalc shotCalc;
  private final String logPrefix;
  private double lastSetpointRad = 0;

  public Hood(int motorID, RobotSide side, ShotCalc shotCalc) {
    this.shotCalc = shotCalc;
    this.logPrefix = "Hood/" + (side == RobotSide.LEFT ? "Left" : "Right");

    motor = new TalonFXS(motorID);

    var motorConfig = new TalonFXSConfiguration();
    motorConfig.MotorOutput =
        new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake);
    motorConfig.CurrentLimits =
        new CurrentLimitsConfigs()
            .withStatorCurrentLimit(Amps.of(40))
            .withStatorCurrentLimitEnable(true);
    // TalonFXS uses integrated rotor (Minion encoder) by default; we scale in code via
    // kEncoderToHoodRadiansPerRotation and kEncoderOffsetRotations.
    motorConfig.MotionMagic =
        new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(RotationsPerSecond.of(kMotionMagicCruiseVelocity))
            .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(kMotionMagicAcceleration))
            .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(100));
    motorConfig.Slot0 =
        new Slot0Configs().withKP(kP).withKI(kI).withKD(kD).withKS(kS).withKV(kV).withKA(kA);

    motorConfig.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;

    motor.getConfigurator().apply(motorConfig);

    // Optional: set position to 0 at boot so offset matches mechanism zero
    motor.getConfigurator().setPosition(kEncoderOffsetRotations);
  }

  /** Sets the hood angle setpoint in radians. Clamped to [kHoodMinAngleRad, kHoodMaxAngleRad]. */
  public void setAngleRad(double angleRad) {
    double clamped = Math.max(kHoodMinAngleRad, Math.min(kHoodMaxAngleRad, angleRad));
    lastSetpointRad = clamped;
    double setpointRotations =
        clamped / kEncoderToHoodRadiansPerRotation + kEncoderOffsetRotations;
    motor.setControl(positionRequest.withPosition(setpointRotations));
  }

  /** Sets the hood angle setpoint in degrees. */
  public void setAngleDegrees(double angleDeg) {
    setAngleRad(Math.toRadians(angleDeg));
  }

  /** Returns the current hood angle in radians (from controller encoder). */
  public double getAngleRad() {
    double positionRotations = motor.getPosition().getValueAsDouble();
    return (positionRotations - kEncoderOffsetRotations) * kEncoderToHoodRadiansPerRotation;
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

  /** Continuously updates setpoint from ShotCalc for hub tracking. Use as default command. */
  public Command trackHub() {
    return run(
        () -> {
          shotCalc.updateState();
          setAngleRad(shotCalc.getPitchForDistance(shotCalc.getXf()));
        });
  }

  /** Holds current position. */
  public Command stopHood() {
    return run(() -> setAngleRad(getAngleRad()));
  }

  @Override
  public void periodic() {
    double angleRad = getAngleRad();
    Logger.recordOutput(logPrefix + "/AngleRad", angleRad);
    Logger.recordOutput(logPrefix + "/AngleDeg", Math.toDegrees(angleRad));
    Logger.recordOutput(logPrefix + "/SetpointRad", lastSetpointRad);
    Logger.recordOutput(logPrefix + "/CurrentAmps", motor.getStatorCurrent().getValueAsDouble());
    Logger.recordOutput(logPrefix + "/Voltage", motor.getMotorVoltage().getValueAsDouble());
  }
}
