package frc.robot.subsystems.Shooters;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;

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
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotSide;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.SimFiles.TurretSim;
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.Logger;

public class Turret extends SubsystemBase {

  private final TalonFXS turretMotor;
  private final MotionMagicVoltage turretRequest = new MotionMagicVoltage(0);
  private final ShotCalc shotCalc;
  private final RobotSide robotSide;
  private final Drive drive;
  private final Transform3d turretOffset;
  private double yawOffset;
  private double targetYaw;
  private final TurretSim simTurret;

  public Turret(
      int leaderID,
      RobotSide robotSide,
      Drive drive,
      Transform3d turretOffset,
      TurretSim simTurret) {

    turretMotor = new TalonFXS(leaderID);
    shotCalc = new ShotCalc(turretOffset);
    this.drive = drive;
    this.turretOffset = turretOffset;
    this.robotSide = robotSide;
    this.simTurret = simTurret;

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
                    .withMotionMagicCruiseVelocity(RotationsPerSecond.of(20))
                    .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(100))
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

    turretMotor.getConfigurator().apply(cfg);

    turretMotor
        .getConfigurator()
        .setPosition(turretSignage(TurretConstants.kInitialTurretPosition));
  }

  public void setYawOffset(double yawOffset) {
    this.yawOffset = yawOffset;
  }

  public Command offsetYaw(double offset) {
    return Commands.run(
        () -> {
          this.setYawOffset(this.yawOffset + offset);
        });
  }

  public void run(double rotations) {
    turretMotor.setControl(turretRequest.withPosition(angleToEncoder(rotations)));
  }

  public Command runCommand(double rotations) {
    return run(
        () -> {
          this.runEncoder(rotations);
        });
  }

  public void runEncoder(double count) {
    turretMotor.setControl(turretRequest.withPosition(count));
  }

  public boolean nearSetpoint(double count) {
    return (Math.abs(turretMotor.getPosition().getValueAsDouble() - count) <= 0.05);
  }

  @Override
  public void periodic() {
    Logger.recordOutput(
        "Turret Encoder Counts" + (robotSide == RobotSide.LEFT ? " Left" : " Right"),
        turretMotor.getPosition().getValueAsDouble());

    Logger.recordOutput("Turret Voltage", turretMotor.getSupplyVoltage().getValueAsDouble());
    Logger.recordOutput(
        "Turret Supply Amperage", turretMotor.getSupplyCurrent().getValueAsDouble());
    Logger.recordOutput(
        "Turret Stator Amperage", turretMotor.getStatorCurrent().getValueAsDouble());

    targetYaw = shotCalc.getRobotRelativeYaw(drive.getPose()) + Math.PI / 2;
    // if (this.robotSide == RobotSide.RIGHT) {
    //   targetYaw = targetYaw - Math.PI;
    // }

    Logger.recordOutput(
        "Suggested Encoder Count" + (robotSide == RobotSide.LEFT ? " Left" : " Right"),
        angleToEncoder(mod(-targetYaw)));

    Logger.recordOutput(
        "Turret Voltage" + (robotSide == RobotSide.LEFT ? " Left" : " Right"),
        turretMotor.getMotorVoltage().getValueAsDouble());
    Logger.recordOutput(
        "Turret Current" + (robotSide == RobotSide.LEFT ? " Left" : " Right"),
        turretMotor.getSupplyCurrent().getValueAsDouble());
  }

  public Command followHub() {
    return run(
        () -> {
          double targetYaw = shotCalc.getRobotRelativeYaw(this.drive.getPose()) - Math.PI / 2;
          if (this.robotSide == RobotSide.RIGHT) {
            targetYaw = targetYaw - Math.PI;
          }
          this.run(targetYaw + yawOffset);
        });
  }

  public double angleToEncoder(double angle) {
    double minEncoderCount = 0;
    double maxEncoderCount = 10;
    double encoderRange = maxEncoderCount - minEncoderCount;
    double angleRange = 2 * Math.PI;

    return ((angle * encoderRange) / angleRange) + minEncoderCount;
  }

  public Command setTurretPos(double angle) {
    return run(() -> {
          this.run(angle);
        })
        .until(() -> nearSetpoint(angleToEncoder(turretSignage(mod(angle)))))
        .andThen(stopTurret());
  }

  public Command setTurretPosEncoder(double encoder) {
    return Commands.run(
        () -> {
          targetYaw = shotCalc.getRobotRelativeYaw(drive.getPose()) + Math.PI / 2;
          // if (this.robotSide == RobotSide.RIGHT) {
          //   targetYaw = targetYaw - Math.PI;
          // }
          this.runEncoder(angleToEncoder(mod(-targetYaw)));
        });
  }

  public Command stopTurret() {
    return run(
        () -> {
          turretMotor.set(0);
        });
  }

  // Helper Function
  public double mod(double angle) {
    return ((angle % (2 * Math.PI)) + (2 * Math.PI)) % (2 * Math.PI);
  }

  public double turretSignage(double value) {
    double signage = this.robotSide == RobotSide.RIGHT ? 1 : -1;
    return value * signage;
  }
}
