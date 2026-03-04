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
                    .withKA(TurretConstants.kA));

    cfg.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;

    turretMotor.getConfigurator().apply(cfg);

    turretMotor.getConfigurator().setPosition(TurretConstants.kInitialTurretPosition);
  }

  public void run(double rotations) {
    turretMotor.setControl(turretRequest.withPosition(angleToEncoder(mod(rotations))));
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Turret Encoder Counts", turretMotor.getPosition().getValueAsDouble());

    shotCalc.updateState();
    double targetYaw = shotCalc.getYaw(drive.getPose());
    Logger.recordOutput("targetYaw", targetYaw);
    this.setTurretPos(targetYaw * (robotSide == RobotSide.RIGHT ? -1 : 1));
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
