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

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.ExternalFeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.ExternalFeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.RobotSide;
import frc.robot.subsystems.SimFiles.TurretSim;
import org.littletonrobotics.junction.Logger;

/**
 * Hood subsystem using a REV Spark MAX + NEO 550. Each hood uses its own internal relative encoder
 * and supports a side-specific zero offset.
 */
public class Hood extends SubsystemBase {
  private final TalonFXS hoodMotor;
  private final CANcoder hoodEncoder;
  private final MotionMagicVoltage hoodRequest = new MotionMagicVoltage(0);
  private final TurretSim simTurret;
  private final RobotSide side;
  private final DutyCycleOut motorDutyCycleOut = new DutyCycleOut(0);

  public Hood(int motorID, int encoderID, RobotSide side, TurretSim simTurret) {

    this.simTurret = simTurret;
    this.side = side;

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
                    // .withRotorToSensorRatio(1)
                    // .withSensorToMechanismRatio(102.47596154)
                    .withFeedbackRemoteSensorID(encoderID)
                    .withExternalFeedbackSensorSource(
                        ExternalFeedbackSensorSourceValue.RemoteCANcoder))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(60))
                    .withStatorCurrentLimitEnable(true))
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(RotationsPerSecond.of(100))
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

    CANcoderConfiguration encCfg =
        new CANcoderConfiguration()
            .withMagnetSensor(
                new MagnetSensorConfigs()
                    .withSensorDirection(SensorDirectionValue.Clockwise_Positive));

    cfg.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;

    hoodEncoder.setPosition(0);

    hoodEncoder.getConfigurator().apply(encCfg);
    hoodMotor.getConfigurator().apply(cfg);

    hoodMotor.getConfigurator().setPosition(HoodConstants.kInitialHoodPosition);
  }

  /**
   * Sets the hood angle setpoint in radians (0 = up, π/2 = horizontal, π = down). Clamped to
   * [kHoodMinAngleRad, kHoodMaxAngleRad].
   */
  public void setAngleRad(double angleRad) {
    double setpointRotations = angleRad / (2 * Math.PI) * 16.73076923;

    // Clamping values
    hoodMotor.setControl(hoodRequest.withPosition(setpointRotations));
  }

  /** Sets the hood angle setpoint in degrees. */
  public void setAngleDegrees(double angleDeg) {
    setAngleRad(Math.toRadians(angleDeg));
  }

  /** Returns the current hood angle in radians (0 = up, π/2 = horizontal, π = down). */
  public double getAngleRad() {
    return hoodMotor.getPosition().getValueAsDouble() * 2 * Math.PI / 16.73076923;
  }

  public Command toAngleRad(double angleRad) {
    return runOnce(() -> setAngleRad(angleRad));
  }

  public Command toAngleDegrees(double angleDeg) {
    return runOnce(() -> setAngleDegrees(angleDeg));
  }

  public Command moveUp() {
    return run(
        () -> {
          hoodMotor.setControl(motorDutyCycleOut.withOutput(0.025));
        });
  }

  public Command moveDown() {
    return run(
        () -> {
          hoodMotor.setControl(motorDutyCycleOut.withOutput(-0.035));
        });
  }

  public Command stop() {
    return run(
        () -> {
          hoodMotor.setControl(motorDutyCycleOut.withOutput(0.0));
        });
  }

  /**
   * Continuously updates setpoint from ShotCalc for hub tracking. Converts launch angle (0 =
   * horizontal, π/2 = up) to hood angle (0 = up, π/2 = horizontal, π = down): hoodAngle = π/2 -
   * launchPitch.
   */
  public Command trackHub() {
    return run(
        () -> {
          double launchPitchRad = simTurret.getMovingPitch();
          setAngleRad(launchPitchRad);
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
    Logger.recordOutput("Hood/AngleRad" + side, angleRad);
    Logger.recordOutput("Hood/AngleDeg" + side, Math.toDegrees(angleRad));
    Logger.recordOutput("Hood/EncoderRotations" + side, rawEncoderRotations);
    Logger.recordOutput(
        "Hood/StatorCurrentAmps" + side, hoodMotor.getStatorCurrent().getValueAsDouble());
    Logger.recordOutput("Hood/Voltage" + side, hoodMotor.getMotorVoltage().getValueAsDouble());
  }
}
