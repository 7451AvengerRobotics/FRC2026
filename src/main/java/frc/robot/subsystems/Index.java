package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;

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
import frc.robot.Constants.IndexConstants;
import frc.robot.Constants.IntakePivotConstants;
import org.littletonrobotics.junction.Logger;

public class Index extends SubsystemBase {

  private final TalonFX indexMotor = new TalonFX(IndexConstants.kIndexID);
  private final DutyCycleOut motorDutyCycleOut = new DutyCycleOut(0);

  public Index() {
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
                    .withStatorCurrentLimit(Amps.of(100))
                    .withStatorCurrentLimitEnable(true));

    indexMotor.getConfigurator().apply(cfg);
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Index Supply Voltage", indexMotor.getSupplyVoltage().getValueAsDouble());
    Logger.recordOutput("Index Supply Amperage", indexMotor.getSupplyCurrent().getValueAsDouble());
    Logger.recordOutput("Index Stator Amperage", indexMotor.getStatorCurrent().getValueAsDouble());

    Logger.recordOutput("Index Motor Voltage", indexMotor.getMotorVoltage().getValueAsDouble());
  }

  public void run(double speed) {
    indexMotor.setControl(motorDutyCycleOut.withOutput(speed));
  }

  public Command runIndex(double power) {
    return run(
        () -> {
          this.run(power);
        });
  }

  public Command stopIndex() {
    return run(
        () -> {
          this.run(0);
        });
  }
}
