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
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakePivotConstants;

public class Intake extends SubsystemBase {

  private final TalonFX intake = new TalonFX(IntakeConstants.kIntakeID);
  private final DutyCycleOut motorDutyCycleOut = new DutyCycleOut(0);
  private final NetworkTable intakeTable = NetworkTableInstance.getDefault().getTable("Intake");

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
                    .withStatorCurrentLimit(Amps.of(40))
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

  @Override
  public void periodic() {
    double velocity = intake.getVelocity().getValueAsDouble();
    double current = intake.getStatorCurrent().getValueAsDouble();
    intakeTable.getEntry("Stall").setBoolean(propIntake());
    intakeTable.getEntry("Velocity").setDouble(velocity);
    intakeTable.getEntry("Current").setDouble(current);
  }
}
