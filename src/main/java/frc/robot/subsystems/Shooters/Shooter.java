package frc.robot.subsystems.Shooters;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
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

public class Shooter extends SubsystemBase {

  private final TalonFX shooterLeader;
  private final TalonFX shooterFollower;
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0);

  public Shooter(int leaderID, int followerID) {

    shooterLeader = new TalonFX(leaderID);
    shooterFollower = new TalonFX(followerID);

    TalonFXConfiguration cfg =
        new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Coast))
            .withFeedback(
                new FeedbackConfigs()
                    .withRotorToSensorRatio(1)
                    .withSensorToMechanismRatio(ShooterConstants.kShooterGearRatio))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(80))
                    .withStatorCurrentLimitEnable(true))
            .withSlot0(
                new Slot0Configs()
                    .withKP(ShooterConstants.kS)
                    .withKI(ShooterConstants.kP)
                    .withKD(ShooterConstants.kV));

    shooterLeader.getConfigurator().apply(cfg);
    shooterFollower.getConfigurator().apply(cfg);

    shooterFollower.setControl(new Follower(followerID, MotorAlignmentValue.Opposed));
  }

  public void run(double targetRPS) {
    shooterLeader.setControl(velocityRequest.withVelocity(targetRPS));
  }

  public Command runShooter() {
    return run(
        () -> {
          this.run(1);
        });
  }

  public Command stopShooter() {
    return run(
        () -> {
          this.run(0);
        });
  }
}
