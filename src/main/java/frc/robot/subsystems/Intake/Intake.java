package frc.robot.subsystems.Intake;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakePivotConstants;

public class Intake extends SubsystemBase {

  private final TalonFX intake = new TalonFX(IntakeConstants.kIntakeID);
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
                    .withStatorCurrentLimit(Amps.of(80))
                    .withStatorCurrentLimitEnable(true));

    intake.getConfigurator().apply(cfg);
  }

  public void setIntakePower(double power) {
    intake.setControl(motorDutyCycleOut.withOutput(power));
  }

  public boolean propIntake() {
    return Math.abs(intake.getVelocity().getValueAsDouble()) < 2
        && (intake.getStatorCurrent().getValueAsDouble() > 50
            && intake.getStatorCurrent().getValueAsDouble() < 60);
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
}
