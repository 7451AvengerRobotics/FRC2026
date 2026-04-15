package frc.robot.subsystems.Intake;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakePivotConstants;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

  private final TalonFX intakeLeader = new TalonFX(IntakeConstants.kIntakeLeaderID);
  private final TalonFX intakeFollower = new TalonFX(IntakeConstants.kIntakeFollowerID);
  private final DutyCycleOut motorDutyCycleOut = new DutyCycleOut(0);

  public Intake() {
    TalonFXConfiguration cfg =
        new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Coast))
            .withFeedback(
                new FeedbackConfigs()
                    .withRotorToSensorRatio(1)
                    .withSensorToMechanismRatio(IntakePivotConstants.kIntakeGearRatio))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(60))
                    .withStatorCurrentLimitEnable(true));

    intakeLeader.getConfigurator().apply(cfg);
    intakeFollower.getConfigurator().apply(cfg);

    intakeFollower.setControl(new Follower(IntakeConstants.kIntakeLeaderID, MotorAlignmentValue.Aligned));
  }

  public void setIntakePower(double power) {
    intakeLeader.setControl(motorDutyCycleOut.withOutput(power));
  }

  public Command runIntake(double power) {
    return run(
        () -> {
          this.setIntakePower(power);
        });
  }

  public Command stopIntake() {
    return run(
        () -> {
          this.setIntakePower(0);
        });
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Intake Voltage", intakeLeader.getMotorVoltage().getValueAsDouble());
    Logger.recordOutput("Intake Current", intakeLeader.getStatorCurrent().getValueAsDouble());
  }
}
