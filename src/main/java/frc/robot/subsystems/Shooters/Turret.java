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
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotSide;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.drive.Drive;

public class Turret extends SubsystemBase {

  private final TalonFXS turretMotor;
  private final MotionMagicVoltage turretRequest = new MotionMagicVoltage(0);
  private final ShotCalc shotCalc;
  private final Drive drive;
  private final RobotSide robotSide;

  public Turret(int leaderID, Drive drive, RobotSide robotSide, ShotCalc shotCalc) {
    turretMotor = new TalonFXS(leaderID);
    this.shotCalc = shotCalc;
    this.drive = drive;
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
                    .withKA(TurretConstants.kA))
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(RotationsPerSecond.of(0.25))
                    .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(5))
                    .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(100)));

    cfg.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;

    turretMotor.getConfigurator().apply(cfg);

    turretMotor.getConfigurator().setPosition(TurretConstants.kInitialTurretPosition);
  }

  public Command setStartingPos() {
    return runOnce(
        () -> {
          turretMotor.getConfigurator().setPosition(TurretConstants.kInitialTurretPosition);
        });
  }

  /** Encoder range [0, 10] corresponds to one full turret revolution [0, 2π] rad. */
  private static final double ENCODER_RANGE = 10.0;

  private static final double ANGLE_RANGE_RAD = 2 * Math.PI;

  /**
   * Commands the turret to the given angle in radians (robot-relative yaw). Use [0, 2π). The
   * controller will move to the corresponding encoder setpoint.
   */
  public void run(double angleRad) {
    turretMotor.setControl(turretRequest.withPosition(angleToEncoder(mod(angleRad))));
  }

  public Command runEncoder(double encoder) {
    return run(() -> turretMotor.setControl(turretRequest.withPosition(encoder)));
  }

  public Command runDutyCycle(double power) {
    return run(
        () -> {
          turretMotor.setControl(new DutyCycleOut(power));
        });
  }

  @Override
  public void periodic() {
    // double encoderPos = turretMotor.getPosition().getValueAsDouble();
    // Logger.recordOutput("Turret Encoder Counts", encoderPos);

    // shotCalc.updateState();
    // Pose2d pose = drive.getPose();
    // double currentTurretRad = encoderToAngleRad(encoderPos);
    // double targetYaw = shotCalc.getYaw(pose);
    // double targetForTurret = (robotSide == RobotSide.RIGHT) ? shotCalc.mod(-targetYaw) :
    // targetYaw;
    // double setpointRad = shotCalc.getYawEquivalentClosestTo(targetForTurret, currentTurretRad);
    // Logger.recordOutput("targetYaw", targetYaw);
    // Logger.recordOutput("Turret/SetpointRad", setpointRad);
    // this.run(setpointRad);
  }

  /** Converts angle in radians [0, 2π) to encoder position [0, ENCODER_RANGE]. */
  public double angleToEncoder(double angleRad) {
    return (mod(angleRad) / ANGLE_RANGE_RAD) * ENCODER_RANGE;
  }

  /** Converts encoder position to angle in radians. */
  private double encoderToAngleRad(double encoderPosition) {
    return (encoderPosition % ENCODER_RANGE) * (ANGLE_RANGE_RAD / ENCODER_RANGE);
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

  /**
   * Returns a command that continuously aims the turret at the hub (shortest path). Use with
   * Commands.parallel() to run both turrets and hoods together for testing.
   */
  public Command trackHubCommand() {
    return run(
        () -> {
          shotCalc.updateState();
          Pose2d pose = drive.getPose();
          double encoderPos = turretMotor.getPosition().getValueAsDouble();
          double currentTurretRad = encoderToAngleRad(encoderPos);
          double targetYaw = shotCalc.getYaw(pose);
          double targetForTurret =
              (robotSide == RobotSide.RIGHT) ? shotCalc.mod(-targetYaw) : targetYaw;
          double setpointRad =
              shotCalc.getYawEquivalentClosestTo(targetForTurret, currentTurretRad);
          this.run(0);
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
