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
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HoodConstants;
import frc.robot.subsystems.SimFiles.TurretSim;
import org.littletonrobotics.junction.Logger;

/**
 * Hood subsystem using a REV Spark MAX + NEO 550. Each hood uses its own internal relative encoder
 * and supports a side-specific zero offset.
 */
public class Hood extends SubsystemBase {
  private final TalonFX hoodMotor;
  private final CANcoder hoodEncoder;
  private final MotionMagicVoltage hoodRequest = new MotionMagicVoltage(0);
  private final TurretSim simTurret;
  private final DutyCycleOut motorDutyCycleOut = new DutyCycleOut(0);

  public Hood(TurretSim simTurret) {

    this.simTurret = simTurret;

    hoodMotor = new TalonFX(40);
    hoodEncoder = new CANcoder(0);

    TalonFXConfiguration cfg =
        new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake))
            .withFeedback(
                new FeedbackConfigs()
                    .withFeedbackRemoteSensorID(HoodConstants.kHoodEncoderID)
                    .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(60))
                    .withStatorCurrentLimitEnable(true))
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(RotationsPerSecond.of(200))
                    .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(100))
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
                    .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive));

    hoodEncoder.setPosition(HoodConstants.kInitialHoodEncoderPosition);

    hoodEncoder.getConfigurator().apply(encCfg);
    hoodMotor.getConfigurator().apply(cfg);

    hoodMotor.getConfigurator().setPosition(HoodConstants.kInitialHoodEncoderPosition);
  }

  public void setAngleRad(double angleRad) {
    double setpointRotations = angleRad / (2 * Math.PI) * HoodConstants.kHoodGearRatio;
    hoodMotor.setControl(hoodRequest.withPosition(setpointRotations));
  }

  public void setAngleDegrees(double angleDeg) {
    setAngleRad(Math.toRadians(angleDeg));
  }

  public double getAngleRad() {
    return hoodMotor.getPosition().getValueAsDouble() * 2 * Math.PI / HoodConstants.kHoodGearRatio;
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
          hoodMotor.setControl(motorDutyCycleOut.withOutput(0.05));
        });
  }

  public Command moveDown() {
    return run(
        () -> {
          hoodMotor.setControl(motorDutyCycleOut.withOutput(-0.05));
        });
  }

  public Command stop() {
    return run(
        () -> {
          hoodMotor.setControl(motorDutyCycleOut.withOutput(0.0));
        });
  }

  public Command trackHub() {
    return run(
        () -> {
          double launchPitchRad = simTurret.getMovingPitch();
          setAngleRad(
              MathUtil.clamp(
                  launchPitchRad,
                  Math.toRadians(HoodConstants.kInitialHoodAnglePosition),
                  Math.toRadians(HoodConstants.kMaxHoodAnglePosition)));
        });
  }

  public Command pass() {
    return run(
        () -> {
          double launchPitchRad = simTurret.getPassingPitch();
          setAngleRad(
              MathUtil.clamp(
                  launchPitchRad,
                  Math.toRadians(HoodConstants.kInitialHoodAnglePosition),
                  Math.toRadians(HoodConstants.kMaxHoodAnglePosition)));
        });
  }

  public Command stopHood() {
    return run(() -> hoodMotor.set(0));
  }

  public Command resetHood() {
    return toAngleDegrees(HoodConstants.kInitialHoodAnglePosition);
  }

  @Override
  public void periodic() {
    double angleRad = getAngleRad();
    Logger.recordOutput("Hood/AngleDeg", Math.toDegrees(angleRad));
    Logger.recordOutput("Hood/EncoderCount", hoodMotor.getPosition().getValueAsDouble());
    Logger.recordOutput("Hood/SuggestedAngleDeg", Math.toDegrees(simTurret.getMovingPitch()));
    Logger.recordOutput("hood Voltage", hoodMotor.getMotorVoltage().getValueAsDouble());
    Logger.recordOutput("hood Current", hoodMotor.getStatorCurrent().getValueAsDouble());
  }
}
