package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
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
import frc.robot.Constants.IndexConstants;
import org.littletonrobotics.junction.Logger;

public class Index extends SubsystemBase {

  private final TalonFX indexLeader = new TalonFX(IndexConstants.kIndexLeaderID);
  private final TalonFX indexFollower = new TalonFX(IndexConstants.kIndexFollowerID);
  private final DutyCycleOut motorDutyCycleOut = new DutyCycleOut(0);

  public Index() {
    TalonFXConfiguration cfg =
        new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Coast))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(130))
                    .withStatorCurrentLimitEnable(true));

    indexLeader.getConfigurator().apply(cfg);
    indexFollower.getConfigurator().apply(cfg);

    indexFollower.setControl(
        new Follower(IndexConstants.kIndexLeaderID, MotorAlignmentValue.Opposed));
  }

  public void run(double speed) {
    indexLeader.setControl(motorDutyCycleOut.withOutput(speed));
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

  @Override
  public void periodic() {
    Logger.recordOutput("Index Voltage", indexLeader.getMotorVoltage().getValueAsDouble());
    Logger.recordOutput("Index Current", indexLeader.getStatorCurrent().getValueAsDouble());
  }
}
