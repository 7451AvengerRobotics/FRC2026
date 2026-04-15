package frc.robot.subsystems.Shooters;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.SimFiles.TurretSim;
import frc.robot.subsystems.drive.Drive;

import static edu.wpi.first.units.Units.Amps;

import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

  private final TalonFX shooterLeader;
  private final TalonFX shooterFollower;
  private final VelocityTorqueCurrentFOC velocityRequest =
      new VelocityTorqueCurrentFOC(0);
  private TurretSim simTurret;
  private Drive drive;

  public Shooter(TurretSim simTurret, Drive drive) {

    this.simTurret = simTurret;
    this.drive = drive;

    shooterLeader = new TalonFX(ShooterConstants.kShooterLeaderID);
    shooterFollower = new TalonFX(ShooterConstants.kShooterFollowerID);

    TalonFXConfiguration cfg =
        new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake))
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

    shooterFollower.setControl(new Follower(ShooterConstants.kShooterLeaderID, MotorAlignmentValue.Aligned));
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Velocity in RPM", shooterLeader.getVelocity().getValueAsDouble());
    Logger.recordOutput("Shooter Voltage", shooterLeader.getStatorCurrent().getValueAsDouble());
    Logger.recordOutput("Shooter Current", shooterLeader.getMotorVoltage().getValueAsDouble());
  }

  public void run(double power) {
    shooterLeader.set(power);
  }

  public void setVel(double rpm) {
    shooterLeader.setControl(
        velocityRequest.withVelocity(rpm / 60));
  }

  public Command setVelCommand(double rpm) {
    return run(
        () -> {
          this.setVel(rpm);
        });
  }

  public Command runShooter(double velOffset) {
    return run(
        () -> {
          
        });
  }

  public Command runShooter5000() {
    return run(
        () -> {
          setVel(5000);
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
