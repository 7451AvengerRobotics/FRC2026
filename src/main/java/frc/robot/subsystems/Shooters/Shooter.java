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
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

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

    globalCfg.inverted(false)
              .idleMode(IdleMode.kCoast);

    globalCfg.smartCurrentLimit(80);

    globalCfg.encoder
   .positionConversionFactor(1.0 / ShooterConstants.kShooterGearRatio)
   .velocityConversionFactor(1.0 / ShooterConstants.kShooterGearRatio);

   /*
     * Configure the closed loop controller. We want to make sure we set the
     * feedback sensor as the primary encoder.
     */
    globalCfg.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // Set PID values for velocity control in slot 1
        .p(ShooterConstants.kP)
        .feedForward
          // kV is now in Volts, so we multiply by the nominal voltage (12V)
          .kV(ShooterConstants.kV)
          .kS(ShooterConstants.kS);

    globalCfg.closedLoop.maxMotion
        // Set MAXMotion parameters for velocity control in slot 1
        .maxAcceleration(500)
        .cruiseVelocity(6000)
        .allowedProfileError(1);


    leaderCfg.apply(globalCfg);
    followerCfg.apply(globalCfg).follow(shooterLeader).inverted(true);

    shooterLeader.configure(leaderCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    shooterFollower.configure(followerCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void run(double targetRPS) {
    closedLoopController.setSetpoint(targetRPS, ControlType.kMAXMotionVelocityControl);
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
