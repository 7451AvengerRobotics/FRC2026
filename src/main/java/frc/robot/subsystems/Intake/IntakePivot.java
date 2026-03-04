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
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakePivotConstants;
import org.littletonrobotics.junction.Logger;

/* TODO
 * 1. Find proper inverted value
 * 2. Confirm coast vs. brake
 * 3. Set gear ratio
 * 4. Find STOW and DEPLOYED positions
 * 5. Tune PID values
 */
public class IntakePivot extends SubsystemBase {
  private final TalonFX intakePivot = new TalonFX(IntakePivotConstants.kIntakePivotID);
  private final MotionMagicVoltage pivotRequest = new MotionMagicVoltage(0);
  private final NetworkTable pivotTable =
      NetworkTableInstance.getDefault().getTable("Intake Pivot");

  public IntakePivot() {
    TalonFXConfiguration cfg =
        new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake))
            .withFeedback(
                new FeedbackConfigs()
                    .withRotorToSensorRatio(1)
                    .withSensorToMechanismRatio(IntakePivotConstants.kIntakeGearRatio))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(40))
                    .withStatorCurrentLimitEnable(true))
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(RotationsPerSecond.of(1.5))
                    .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(7))
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

    intakePivot.getConfigurator().apply(cfg);

    intakePivot.getConfigurator().setPosition(0);
  }

  public void pivotIntake(double rotations) {
    intakePivot.setControl(pivotRequest.withPosition(rotations));
  }

  public boolean nearSetpoint(double rotations) {
    double diff = intakePivot.getPosition().getValueAsDouble() - rotations;
    return Math.abs(diff) <= 0.05;
  }

  // public boolean atStow() {
  //   return nearSetpoint(PivotPosition.STOW);
  // }

  // public boolean atDeployed() {
  //   return nearSetpoint(PivotPosition.DEPLOYED);
  // }

  public Command setIntakePivotAngle(double rotations) {
    return run(
        () -> {
          pivotIntake(rotations);
        });
  }

  public Command toPosition(double rotations) {
    return setIntakePivotAngle(rotations).until(() -> nearSetpoint(rotations));
  }

  @Override
  public void periodic() {
    Logger.recordOutput("IntakePivot Rotations", intakePivot.getPosition().getValueAsDouble());

    pivotTable
        .getEntry("IntakePivot Rotations")
        .setDouble(intakePivot.getPosition().getValueAsDouble());
  }

  public enum PivotPosition {
    STOW(0),
    DEPLOYED(0.27);

    public final double rotations;

    private PivotPosition(double rotations) {
      this.rotations = rotations;
    }
  }
}
