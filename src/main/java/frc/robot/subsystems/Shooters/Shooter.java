package frc.robot.subsystems.Shooters;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.SimFiles.TurretSim;
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

  private final TalonFX shooterLeader;
  private final TalonFX shooterFollower;
  private final TalonFX shooterMini;
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0);
  private TurretSim simTurret;
  private Drive drive;

  public Shooter(TurretSim simTurret, Drive drive) {

    this.simTurret = simTurret;
    this.drive = drive;

    shooterLeader = new TalonFX(ShooterConstants.kShooterLeaderID);
    shooterFollower = new TalonFX(ShooterConstants.kShooterFollowerID);
    shooterMini = new TalonFX(ShooterConstants.kShooterMiniID);

    TalonFXConfiguration cfg =
        new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Coast))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(60))
                    .withStatorCurrentLimitEnable(true))
            .withSlot0(
                new Slot0Configs()
                    .withKP(ShooterConstants.kP)
                    .withKI(ShooterConstants.kI)
                    .withKD(ShooterConstants.kD)
                    .withKS(ShooterConstants.kS)
                    .withKV(ShooterConstants.kV)
                    .withKA(ShooterConstants.kA));

    shooterLeader.getConfigurator().apply(cfg);
    shooterFollower.getConfigurator().apply(cfg);

    shooterFollower.setControl(
        new Follower(ShooterConstants.kShooterLeaderID, MotorAlignmentValue.Opposed));
    // shooterMini.setControl(
    //     new Follower(ShooterConstants.kShooterLeaderID, MotorAlignmentValue.Aligned));
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Velocity in RPM", shooterLeader.getVelocity().getValueAsDouble() * 60);
    Logger.recordOutput("Shooter Voltage", shooterLeader.getStatorCurrent().getValueAsDouble());
    Logger.recordOutput("Shooter Current", shooterLeader.getMotorVoltage().getValueAsDouble());
    Logger.recordOutput("Mini Voltage", shooterMini.getStatorCurrent().getValueAsDouble());
    Logger.recordOutput("Mini Current", shooterMini.getMotorVoltage().getValueAsDouble());
  }

  public void run(double power) {
    shooterLeader.set(power);
  }

  public Command runDutyCycle(double power) {
    return run(
        () -> {
          this.run(power);
        });
  }

  public void setVel(double rpm) {
    shooterLeader.setControl(velocityRequest.withVelocity(rpm / 60));
  }

  public Command setVelCommand(double rpm) {
    return run(
        () -> {
          this.setVel(rpm);
        });
  }

  public Command runShooter4000() {
    return run(
        () -> {
          setVel(4000);
        });
  }

  public Command runShooter3000() {
    return run(
        () -> {
          setVel(3000);
        });
  }

  public Command stopShooter() {
    return run(
        () -> {
          this.run(0);
        });
  }
}
