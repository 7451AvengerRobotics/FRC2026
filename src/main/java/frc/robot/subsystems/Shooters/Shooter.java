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
  private final SparkClosedLoopController closedLoopController;
  private final ShotCalc shotCalc;
  private final String name;
  private TurretSim simTurret;
  private Drive drive;
  private double velOffset = 1;

  private double ballRequiredVel;

  public Shooter(int leaderID, String name, TurretSim simTurret, Drive drive) {

    shooterLeader = new SparkFlex(leaderID, MotorType.kBrushless);
    this.name = name;
    this.simTurret = simTurret;
    this.drive = drive;
    closedLoopController = shooterLeader.getClosedLoopController();
    this.shotCalc = new ShotCalc(simTurret.getTurretOffset());

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
        // Set MAXMotion parameters for velocity control in slot 1
        .maxAcceleration(2000)
        .allowedProfileError(1);

    leaderCfg.apply(globalCfg).disableFollowerMode();

    shooterLeader.configure(
        leaderCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
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
    Logger.recordOutput("Velocity in RPM_" + name, shooterLeader.getEncoder().getVelocity());
    Logger.recordOutput("Voltage in Volts_" + name, shooterLeader.getAppliedOutput());
    Logger.recordOutput("Current in Amps_" + name, shooterLeader.getOutputCurrent());

    ballRequiredVel = simTurret.getRequiredVelocity();
    Logger.recordOutput("Required Velocity" + name, ballRequiredVel);
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

  public Command runShooter(double velOffset) {
    return run(
        () -> {
          //   double target = Constants.TargetConstants.hub;
          //   double xf =
          // Math.sqrt(
          //     Math.pow(
          //             (target.getX() - turretPositionPose2d.getX()) + vxr *
          // TurretConstants.latency,
          //             2)
          //         + Math.pow(
          //             (target.getY() - turretPositionPose2d.getY()) + vyr *
          // TurretConstants.latency,
          //             2));
          // double numerator = g * Math.pow(xf, 2);
          // double denom1 = -yf + xf * Math.tan(pitch);
          // double denom2 = 2 * Math.pow(Math.cos(pitch), 2);
          //  Math.sqrt(numerator / (denom1 * denom2));
          ballRequiredVel = simTurret.getColumbusVelocity();

          // Compute flywheel target
          double velocityRequired = ballRequiredVel;

          double a = 0.368653;
          double b = 2.13185;
          double c = -0.526856;

          a = 0.103284;
          b = 3.2632;
          // c = 0;
          // double flywheelVel =
          //     (a * Math.pow(velocityRequired, 2) + b * velocityRequired)
          //         * 60
          //         / (2 * Math.PI * 4 * 0.0254);

          double a2 = 4.4763;
          a2 = 4;
          double flywheelVel =
              ((a) * Math.pow(velocityRequired, 2) + b * velocityRequired
                  // + c * RobotController.getBatteryVoltage()
                  )
                  * 60
                  / (2 * Math.PI * 4 * 0.0254);

          double battery = RobotController.getBatteryVoltage();
          double factor = -0.1333 * battery + 2.3663;

          // Optional: clamp to prevent crazy values
          factor = MathUtil.clamp(factor, 0.8, 1.4);
          flywheelVel = MathUtil.clamp(flywheelVel, 0, 5000);

          // Command the motor
          setVel(flywheelVel * velOffset * 1.05);
        });
  }

  public Command runShooter5000() {
    return run(
        () -> {
          setVel(4000);
        });
  }

  public Command runShooter3000() {
    return run(
        () -> {
          setVel(3500);
        });
  }

  // public double flywheelVel() {
  //   double velocityRequired = ballRequiredVel;
  //   double a = -0.123001;
  //   double b = 5.95629;
  //   double flywheelVel =
  //       (a * Math.pow(velocityRequired, 2) + b * velocityRequired)
  //           * 60
  //           / (2 * Math.PI * 4 * 0.0254);
  //   return flywheelVel * 1.2;
  // }

  public Command stopShooter() {
    return run(
        () -> {
          this.run(0);
        });
  }
}
