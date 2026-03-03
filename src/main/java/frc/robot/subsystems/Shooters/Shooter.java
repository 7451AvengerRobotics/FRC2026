package frc.robot.subsystems.Shooters;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.SimFiles.TurretSim;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

  private final SparkFlex shooterLeader;
  private final SparkFlex shooterFollower;
  private final SparkClosedLoopController closedLoopController;
  private final ShotCalc shotCalc = new ShotCalc();
  private final String name;
  private final TurretSim simTurret;

  private GenericEntry speed = Shuffleboard.getTab("Flywheel").add("Speed", 0).getEntry();

  public Shooter(int leaderID, int followerID, String name, TurretSim simTurret) {

    shooterLeader = new SparkFlex(leaderID, MotorType.kBrushless);
    shooterFollower = new SparkFlex(followerID, MotorType.kBrushless);
    this.name = name;
    this.simTurret = simTurret;

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

    globalCfg.smartCurrentLimit(80);

    globalCfg.closedLoop.p(ShooterConstants.kP).d(ShooterConstants.kD);
    globalCfg.closedLoop.feedForward.kS(ShooterConstants.kS).kV(ShooterConstants.kV);

    globalCfg
        .closedLoop
        .maxMotion
        // Set MAXMotion parameters for velocity control in slot 1
        .maxAcceleration(2000)
        .allowedProfileError(1);

    leaderCfg.apply(globalCfg).disableFollowerMode();
    followerCfg.apply(globalCfg).follow(shooterLeader, true);

    shooterLeader.configure(
        leaderCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    shooterFollower.configure(
        followerCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Velocity in RPM_" + name, shooterLeader.getEncoder().getVelocity());
    Logger.recordOutput("Voltage in Volts_" + name, shooterLeader.getAppliedOutput());
    Logger.recordOutput("Current in Amps_" + name, shooterLeader.getOutputCurrent());
    Logger.recordOutput("Flywheel" + name, this.flywheelVel());
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
        });
  }

  public Command runShooter() {
    double velocityRequired = simTurret.getRequiredVelocity();
    double a = -0.0773318;
    double b = 5.49489;
    double flywheelVel =
        (a * Math.pow(velocityRequired, 2) + b * velocityRequired)
            * 60
            / (2 * Math.PI * 4 * 0.0254);
    MathUtil.clamp(flywheelVel, 0, 5000);
    return setVelCommand(speed.getDouble(3000));
  } 

  public double flywheelVel() {
    double velocityRequired = simTurret.getRequiredVelocity();
    double a = -0.0773318;
    double b = 5.49489;
    double flywheelVel =
        (a * Math.pow(velocityRequired, 2) + b * velocityRequired)
            * 60
            / (2 * Math.PI * 4 * 0.0254);
    return flywheelVel;
  }

  public Command stopShooter() {
    return run(
        () -> {
          this.run(0);
        });
  }
}
