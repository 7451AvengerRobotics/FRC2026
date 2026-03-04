package frc.robot.subsystems.Shooters;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotSide;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.Logger;

public class Turret extends SubsystemBase {

  private final TalonFXS turretMotor;
  private final MotionMagicVoltage turretRequest = new MotionMagicVoltage(0);
  private final ShotCalc shotCalc;
  private final RobotSide robotSide;
  private final Drive drive;
  private final Transform3d turretOffset;

  public Turret(int leaderID, RobotSide robotSide, Drive drive, Transform3d turretOffset) {

    turretMotor = new TalonFXS(leaderID);
    shotCalc = new ShotCalc(turretOffset);
    this.drive = drive;
    this.turretOffset = turretOffset;
    this.robotSide = robotSide;

    TalonFXSConfiguration cfg =
        new TalonFXSConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Coast))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(40))
                    .withStatorCurrentLimitEnable(true))
            .withSlot0(
                new Slot0Configs()
                    .withKP(TurretConstants.kP)
                    .withKI(TurretConstants.kI)
                    .withKD(TurretConstants.kD)
                    .withKS(TurretConstants.kS)
                    .withKV(TurretConstants.kV)
                    .withKA(TurretConstants.kA));

    cfg.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;

    turretMotor.getConfigurator().apply(cfg);

    turretMotor.getConfigurator().setPosition((this.robotSide == RobotSide.RIGHT) ? TurretConstants.kInitialTurretPosition : -TurretConstants.kInitialTurretPosition);
  }

  public void run(double rotations) {
    turretMotor.setControl(turretRequest.withPosition(angleToEncoder(mod(rotations))));
  }

  @Override
  public void periodic() {
    Logger.recordOutput(
        "Turret Encoder Counts" + (robotSide == RobotSide.LEFT ? " Left" : " Right"),
        turretMotor.getPosition().getValueAsDouble());

    double targetYaw = shotCalc.getRobotRelativeYaw(this.drive.getPose()) - Math.PI / 2;
    if (this.robotSide == RobotSide.RIGHT) {
      targetYaw = targetYaw + Math.PI;
    }

    Logger.recordOutput(
        "Suggested Encoder Count" + (robotSide == RobotSide.LEFT ? " Left" : " Right"), targetYaw);
  }

  public Command followHub() {
    return run(
        () -> {
          double targetYaw = shotCalc.getRobotRelativeYaw(this.drive.getPose()) - Math.PI / 2;
          if (this.robotSide == RobotSide.RIGHT) {
            targetYaw = targetYaw + Math.PI;
          }
          this.run(targetYaw);
        });
  }

  public double angleToEncoder(double angle) {
    double minEncoderCount = 0;
    double maxEncoderCount = 10;
    double encoderRange = maxEncoderCount - minEncoderCount;
    double angleRange = 2 * Math.PI;

    return ((angle * encoderRange) / angleRange) + minEncoderCount;
  }

  // public Command followHub() {
  //   double targetYaw = shotCalc.getYaw(drive.getPose());
  //   double currentYaw = turretMotor.getPosition().getValueAsDouble();
  // }

  public Command setTurretPos(double angle) {
    return run(
        () -> {
          this.run(angle);
        });
  }

  public Command stopTurret() {
    return run(
        () -> {
          this.run(0);
        });
  }

  // Helper Function
  public double mod(double angle) {
    return ((angle % (2 * Math.PI)) + (2 * Math.PI)) % (2 * Math.PI);
  }
}
