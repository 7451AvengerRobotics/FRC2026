package frc.robot.subsystems.Shooters;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

  private final SparkFlex shooterLeader;
  private final SparkFlex shooterFollower;
  private final SparkClosedLoopController closedLoopController;

  public Shooter(int leaderID, int followerID) {

    shooterLeader = new SparkFlex(leaderID, MotorType.kBrushless);
    shooterFollower = new SparkFlex(followerID, MotorType.kBrushless);

    closedLoopController = shooterLeader.getClosedLoopController();

    // TalonFXConfiguration cfg =
    //     new TalonFXConfiguration()
    //         .withMotorOutput(
    //             new MotorOutputConfigs()
    //                 .withInverted(InvertedValue.CounterClockwise_Positive)
    //                 .withNeutralMode(NeutralModeValue.Coast))
    //         .withFeedback(
    //             new FeedbackConfigs()
    //                 .withRotorToSensorRatio(1)
    //                 .withSensorToMechanismRatio(ShooterConstants.kShooterGearRatio))
    //         .withCurrentLimits(
    //             new CurrentLimitsConfigs()
    //                 .withStatorCurrentLimit(Amps.of(80))
    //                 .withStatorCurrentLimitEnable(true))
    //         .withSlot0(
    //             new Slot0Configs()
    //                 .withKP(ShooterConstants.kS)
    //                 .withKI(ShooterConstants.kP)
    //                 .withKD(ShooterConstants.kV));

    SparkFlexConfig globalCfg = new SparkFlexConfig();
    SparkFlexConfig leaderCfg = new SparkFlexConfig();
    SparkFlexConfig followerCfg = new SparkFlexConfig();

    globalCfg.idleMode(IdleMode.kCoast);

    globalCfg.smartCurrentLimit(35);

    globalCfg.closedLoop.p(ShooterConstants.kP);

    globalCfg
        .closedLoop
        .maxMotion
        // Set MAXMotion parameters for velocity control in slot 1
        .maxAcceleration(500)
        .allowedProfileError(1);

    leaderCfg.apply(globalCfg);
    // followerCfg.apply(globalCfg).follow(shooterLeader).inverted(true);

    shooterLeader.configure(
        leaderCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    shooterFollower.configure(
        followerCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Velocity in RPM", shooterLeader.getEncoder().getVelocity());
  }

  public void run(double power) {
    // closedLoopController.setSetpoint(targetRPM, ControlType.kVelocity);
    shooterLeader.set(power);
  }

  public void setVel(double rpm) {
    closedLoopController.setSetpoint(rpm, ControlType.kMAXMotionVelocityControl);
  }

  public Command setVelCommand(double rpm) {
    return run(
      () -> {
        this.setVel(rpm);
      }
    );
  }

  public Command runShooter() {
    return run(
        () -> {
          this.run(0.9);
        });
  }

  public Command stopShooter() {
    return run(
        () -> {
          this.run(0);
        });
  }
}
