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
import frc.robot.Constants.FeederConstants;
import frc.robot.Constants.IntakePivotConstants;

public class Feeder extends SubsystemBase {

  private final TalonFX feederMotor = new TalonFX(FeederConstants.kFeederID);
  private final DutyCycleOut motorDutyCycleOut = new DutyCycleOut(0);

  public Feeder() {
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

    feederMotor.getConfigurator().apply(cfg);
  }

  public void run(double speed) {
    feederMotor.setControl(motorDutyCycleOut.withOutput(speed));
  }

  public Command runFeeder() {
    return run(
        () -> {
          this.run(1);
        });
  }

  public Command stopFeeder() {
    return run(
        () -> {
          this.run(0);
        });
  }
}
