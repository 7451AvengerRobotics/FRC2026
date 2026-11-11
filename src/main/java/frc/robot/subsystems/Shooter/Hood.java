// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;

import java.util.Set;

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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.TargetConstants;
import frc.robot.subsystems.drive.Drive;

import org.littletonrobotics.junction.Logger;

public class Hood extends SubsystemBase {
    //Make the motors and encoders
    private final TalonFX hoodMotor = new TalonFX(40);
    private final CANcoder hoodEncoder = new CANcoder(0);

    //Make the control request
    private final MotionMagicVoltage hoodRequest = new MotionMagicVoltage(0);

    //Drive reference
    private final Drive drive;

    public Hood(Drive drive) {
        //Make a configuration (settings) for your motor
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
                        .withStatorCurrentLimitEnable(true)
                        .withSupplyCurrentLimit(Amps.of(30))
                        .withSupplyCurrentLimitEnable(true))
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

        //Apply the configuration to the motor's configurator
        hoodEncoder.getConfigurator().apply(encCfg);
        hoodMotor.getConfigurator().apply(cfg);

        // Set the initial position of the hood encoder and motor
        hoodEncoder.setPosition(HoodConstants.kInitialHoodEncoderPosition);
        hoodMotor.getConfigurator().setPosition(HoodConstants.kInitialHoodEncoderPosition);

        this.drive = drive;
    }

    // Make methods to set and get the hood angle in radians and degrees
    public void setAngleRad(double angleRad) {
        double setpointRotations = angleRad / (2 * Math.PI) * HoodConstants.kHoodGearRatio;
        // if (Math.abs(Math.toDegrees(angleRad - getAngleRad())) < 0.01) {
        //     hoodMotor.setControl(new DutyCycleOut(0));
        // } else {
            hoodMotor.setControl(hoodRequest.withPosition(setpointRotations));
        // }
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
        return run(() -> setAngleDegrees(angleDeg));
    }

    public Command moveUp() {
        return run(
            () -> {
            hoodMotor.setControl(new DutyCycleOut(0.05));
            });
    }

    public Command moveDown() {
        return run(
            () -> {
            hoodMotor.setControl(new DutyCycleOut(-0.05));
            });
    }

    public Command stop() {
        return run(
            () -> {
            hoodMotor.setControl(new DutyCycleOut(0.0));
            });
    }

    public Command trackHub() {
        return Commands.defer(
            () -> {
            double launchPitchRad = ShotCalc.newGetPitch(getXf());
            return toAngleRad(
                MathUtil.clamp(
                    launchPitchRad,
                    Math.toRadians(HoodConstants.kInitialHoodAnglePosition),
                    Math.toRadians(HoodConstants.kMaxHoodAnglePosition)));
            }, Set.of(this));
    }

    public Command resetHood() {
        return toAngleDegrees(HoodConstants.kInitialHoodAnglePosition);
    }

    public double getXf() {
        Pose2d shooterPositionPose2d = drive.getPose().plus(TargetConstants.shooterOffset);
        double xf =
            Math.sqrt(
                Math.pow(TargetConstants.hub.getX() - shooterPositionPose2d.getX(), 2)
                    + Math.pow(TargetConstants.hub.getY() - shooterPositionPose2d.getY(), 2));
        return xf;
    }

    @Override
    public void periodic() {
        double angleRad = getAngleRad();
        Logger.recordOutput("Hood/AngleDeg", Math.toDegrees(angleRad));
        Logger.recordOutput("Hood/EncoderCount", hoodMotor.getPosition().getValueAsDouble());
        Logger.recordOutput("hood Voltage", hoodMotor.getMotorVoltage().getValueAsDouble());
        Logger.recordOutput("hood Current", hoodMotor.getStatorCurrent().getValueAsDouble());
    }
}