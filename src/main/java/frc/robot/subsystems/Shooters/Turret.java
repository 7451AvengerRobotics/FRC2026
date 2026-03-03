package frc.robot.subsystems.Shooters;

import static edu.wpi.first.units.Units.Amps;

import org.littletonrobotics.junction.Logger;

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
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.drive.Drive;

public class Turret extends SubsystemBase {

  private final TalonFXS turretMotor;
  private final MotionMagicVoltage turretRequest = new MotionMagicVoltage(0);
  private final ShotCalc shotCalc;

  public Turret(int leaderID) {

    turretMotor = new TalonFXS(leaderID);
    shotCalc = new ShotCalc();

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
  }

  public double angleToEncoder(double angle) {
    double minEncoderCount = 0;
    double maxEncoderCount = 0;
    double encoderRange = maxEncoderCount-minEncoderCount;
    double angleRange = 2*Math.PI;

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
