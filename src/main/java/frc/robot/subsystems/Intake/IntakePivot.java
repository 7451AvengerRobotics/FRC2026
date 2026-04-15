package frc.robot.subsystems.Intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakePivotConstants;
import org.littletonrobotics.junction.Logger;

public class IntakePivot extends SubsystemBase {
  private final TalonFX pivotLeader = new TalonFX(IntakePivotConstants.kPivotLeaderID);
  private final TalonFX pivotFollower = new TalonFX(IntakePivotConstants.kPivotFollowerID);
  private final MotionMagicVoltage pivotRequest = new MotionMagicVoltage(0);

  public IntakePivot() {
    TalonFXConfiguration cfg =
        new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake))
            .withFeedback(
                new FeedbackConfigs()
                    .withSensorToMechanismRatio(IntakePivotConstants.kIntakeGearRatio))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(40))
                    .withStatorCurrentLimitEnable(true))
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(RotationsPerSecond.of(10))
                    .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(10))
                    .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(100)))
            .withSlot0(
                new Slot0Configs()
                    .withKP(IntakePivotConstants.kP)
                    .withKI(IntakePivotConstants.kI)
                    .withKD(IntakePivotConstants.kD)
                    .withKG(IntakePivotConstants.kG)
                    .withKS(IntakePivotConstants.kS)
                    .withKV(IntakePivotConstants.kV)
                    .withKA(IntakePivotConstants.kA));

    pivotLeader.getConfigurator().apply(cfg);
    pivotFollower.getConfigurator().apply(cfg);

    pivotFollower.setControl(new Follower(IntakePivotConstants.kPivotLeaderID, MotorAlignmentValue.Opposed));

    pivotLeader.getConfigurator().setPosition(0);
  }

  public void pivotIntake(double rotations) {
    pivotLeader.setControl(pivotRequest.withPosition(rotations));
  }

  public Command runPivot(double value) {
    return run(() -> pivotLeader.setControl(new DutyCycleOut(value)));
  }

  public boolean nearSetpoint(double rotations) {
    double diff = pivotLeader.getPosition().getValueAsDouble() - rotations;
    return Math.abs(diff) <= 0.1;
  }

  public Command setIntakePivotAngle(double rotations) {
    return run(
        () -> {
          pivotIntake(rotations);
        });
  }

  public Command jiggle() {
    return runPivot(-0.15);
  }

  public Command stopPivot() {
    return runPivot(0);
  }

  public Command toPosition(double rotations) {
    return setIntakePivotAngle(rotations).until(() -> nearSetpoint(rotations));
  }

  public enum PivotPosition {
    STOW(0),
    DEPLOYED(4.2);

    public final double rotations;

    private PivotPosition(double rotations) {
      this.rotations = rotations;
    }
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Pivot Voltage", pivotLeader.getMotorVoltage().getValueAsDouble());
    Logger.recordOutput("Pivot Current", pivotLeader.getStatorCurrent().getValueAsDouble());
  }
}
