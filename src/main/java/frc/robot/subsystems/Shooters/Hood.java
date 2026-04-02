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
import com.ctre.phoenix6.configs.ExternalFeedbackConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.ExternalFeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
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
  private TalonFXS hoodMotor;
  private CANcoder hoodEncoder;
  private final MotionMagicVoltage hoodRequest = new MotionMagicVoltage(0);
  private final TurretSim simTurret;

  public Hood(int motorID, int encoderID, RobotSide side, TurretSim simTurret) {

    this.simTurret = simTurret;

    hoodMotor = new TalonFXS(motorID);
    hoodEncoder = new CANcoder(encoderID);

    TalonFXSConfiguration cfg =
        new TalonFXSConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake))
            .withExternalFeedback(
                new ExternalFeedbackConfigs()
                    .withRotorToSensorRatio(1)
                    .withSensorToMechanismRatio(1)
                    .withFeedbackRemoteSensorID(encoderID)
                    .withExternalFeedbackSensorSource(ExternalFeedbackSensorSourceValue.RemoteCANcoder))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(40))
                    .withStatorCurrentLimitEnable(true))
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(RotationsPerSecond.of(5))
                    .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(75))
                    .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(100)))
            .withSlot0(
                new Slot0Configs()
                    .withKP(HoodConstants.kP)
                    .withKI(HoodConstants.kI)
                    .withKD(HoodConstants.kD)
                    .withKS(HoodConstants.kS)
                    .withKV(HoodConstants.kV)
                    .withKA(HoodConstants.kA)
                    .withKG(HoodConstants.kG));

    cfg.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;

    hoodMotor.getConfigurator().apply(cfg);

    hoodMotor.getConfigurator().setPosition(HoodConstants.kInitialHoodPosition);
  }

  /**
   * Sets the hood angle setpoint in radians (0 = up, π/2 = horizontal, π = down). Clamped to
   * [kHoodMinAngleRad, kHoodMaxAngleRad].
   */
  public void setAngleRad(double angleRad) {
    double setpointRotations = angleRad / (2 * Math.PI);
    hoodMotor.setControl(hoodRequest.withPosition(setpointRotations));
  }

  /** Sets the hood angle setpoint in degrees. */
  public void setAngleDegrees(double angleDeg) {
    setAngleRad(Math.toRadians(angleDeg));
  }

  /** Returns the current hood angle in radians (0 = up, π/2 = horizontal, π = down). */
  public double getAngleRad() {
    return hoodMotor.getPosition().getValueAsDouble() * 2 * Math.PI;
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

  /** Cuts power */
  public Command stopHood() {
    return run(() -> hoodMotor.set(0));
  }

  @Override
  public void periodic() {
    double angleRad = getAngleRad();
    double rawEncoderRotations = hoodMotor.getPosition().getValueAsDouble();
    Logger.recordOutput("Hood/AngleRad", angleRad);
    Logger.recordOutput("Hood/AngleDeg", Math.toDegrees(angleRad));
    Logger.recordOutput("Hood/EncoderRotations", rawEncoderRotations);
    Logger.recordOutput("Hood/StatorCurrentAmps", hoodMotor.getStatorCurrent().getValueAsDouble());
    Logger.recordOutput("Hood/Voltage", hoodMotor.getMotorVoltage().getValueAsDouble());
  }
}
