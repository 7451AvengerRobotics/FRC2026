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
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakePivotConstants;

public class IntakePivot extends SubsystemBase {
    
    //Make the motors
    private final TalonFX pivotLeaderMotor = new TalonFX(IntakePivotConstants.kPivotLeaderID);
    private final TalonFX pivotFollowerMotor = new TalonFX(IntakePivotConstants.kPivotFollowerID);
    
    //Make the control request
    private final MotionMagicVoltage pivotRequest = new MotionMagicVoltage(0);

    public IntakePivot() {

        //Make a configuration (settings) for your motor
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
                        .withStatorCurrentLimitEnable(true)
                        .withSupplyCurrentLimit(Amps.of(30))
                        .withSupplyCurrentLimitEnable(true))
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
                        .withKS(IntakePivotConstants.kS)
                        .withKV(IntakePivotConstants.kV)
                        .withKA(IntakePivotConstants.kA)
                        .withKG(IntakePivotConstants.kG)
                        .withGravityType(GravityTypeValue.Arm_Cosine));

        //Apply the configuration to the motor's configurator
        pivotLeaderMotor.getConfigurator().apply(cfg);
        pivotFollowerMotor.getConfigurator().apply(cfg);
        
        //Apply any follower controls
        pivotFollowerMotor.setControl(new Follower(IntakePivotConstants.kPivotLeaderID, MotorAlignmentValue.Opposed));
        
        //Extra controls
        pivotLeaderMotor.getConfigurator().setPosition(IntakePivotConstants.kInitialPivotPosition);
    }

    //Make a method to power the motor
    public void movePivotMethod(double position) {
        pivotLeaderMotor.setControl(pivotRequest.withPosition(position));
    }

    public void powerPivotMethod(double power) {
        pivotLeaderMotor.setControl(new DutyCycleOut(power));
    }

    //Make a command to power the motor
    public Command movePivot(double position) {
        return run(
            () -> {
                this.movePivotMethod(position);
            }
        );
    }

    public Command powerIntake(double power) {
        return run(
            () -> {
                this.powerPivotMethod(power);
            }
        );
    }

    //Extra commands
    public Command stopPivot() {
        return powerIntake(0);
    }

}
