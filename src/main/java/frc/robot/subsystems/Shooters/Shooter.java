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
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.SimFiles.TurretSim;
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

  private final SparkFlex shooterLeader;
  private final SparkFlex shooterFollower;
  private final SparkClosedLoopController closedLoopController;
  private final ShotCalc shotCalc;
  private TurretSim simTurret;
  private Drive drive;
  private double velOffset = 1;

  private double ballRequiredVel;

  public Shooter(int leaderID, int followerID, TurretSim simTurret, Drive drive) {

    shooterLeader = new SparkFlex(leaderID, MotorType.kBrushless);
    shooterFollower = new SparkFlex(followerID, MotorType.kBrushless);
    this.simTurret = simTurret;
    this.drive = drive;
    closedLoopController = shooterLeader.getClosedLoopController();
    this.shotCalc = new ShotCalc(simTurret.getTurretOffset());

    SparkFlexConfig globalCfg = new SparkFlexConfig();
    SparkFlexConfig leaderCfg = new SparkFlexConfig();
    SparkFlexConfig followerCfg = new SparkFlexConfig();

    globalCfg.idleMode(IdleMode.kCoast);

    globalCfg.smartCurrentLimit(40);
    globalCfg.voltageCompensation(12.0);

    globalCfg.closedLoop.p(ShooterConstants.kP).d(ShooterConstants.kD);
    globalCfg
        .closedLoop
        .feedForward
        .kS(ShooterConstants.kS)
        .kV(ShooterConstants.kV)
        .kA(ShooterConstants.kA);

    globalCfg
        .closedLoop
        .maxMotion
          .maxAcceleration(2000)
          .allowedProfileError(1);

    leaderCfg.apply(globalCfg).disableFollowerMode();
    followerCfg.apply(globalCfg).follow(shooterLeader, true);

    shooterLeader.configure(
        leaderCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    shooterFollower.configure(
        followerCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setVelOffset(double offset) {
    this.velOffset = offset;
  }

  public double getVelOffset() {
    return this.velOffset;
  }

  public double getRPM() {
    return shooterLeader.getEncoder().getVelocity();
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Velocity in RPM", shooterLeader.getEncoder().getVelocity());
    Logger.recordOutput("Shooter Voltage", shooterLeader.getAppliedOutput());
    Logger.recordOutput("Shooter Current", shooterLeader.getOutputCurrent());
  }

  public void run(double power) {
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

  public Command runShooter(double velOffset) {
    return run(
        () -> {
          ballRequiredVel = simTurret.getColumbusVelocity();

          double velocityRequired = ballRequiredVel;

          double a = 0.368653;
          double b = 2.13185;
          double c = -0.526856;

          a = 0.103284;
          b = 3.2632;

          double a2 = 4.4763;
          a2 = 4;
          double flywheelVel =
              (a * Math.pow(velocityRequired, 2) + b * velocityRequired)
                  * 60
                  / (2 * Math.PI * 4 * 0.0254);

          double battery = RobotController.getBatteryVoltage();
          double factor = -0.1333 * battery + 2.3663;

          factor = MathUtil.clamp(factor, 0.8, 1.4);
          flywheelVel = MathUtil.clamp(flywheelVel, 0, 5000);

          setVel(flywheelVel * velOffset * 1.05);
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
