package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;

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

public class Intake extends SubsystemBase {
    
    //Make the motors
    private final TalonFX intakeLeaderMotor = new TalonFX(IntakeConstants.kIntakeLeaderID);
    private final TalonFX intakeFollowerMotor = new TalonFX(IntakeConstants.kIntakeFollowerID);
    
    //Make the control request
    private final DutyCycleOut intakeDutyCycle = new DutyCycleOut(0);

    public Intake() {

        //Make a configuration (settings) for your motor
        TalonFXConfiguration cfg = 
            new TalonFXConfiguration()
                .withMotorOutput(
                    new MotorOutputConfigs()
                        .withInverted(InvertedValue.CounterClockwise_Positive)
                        .withNeutralMode(NeutralModeValue.Coast))
                .withFeedback(
                    new FeedbackConfigs()
                        .withRotorToSensorRatio(1)
                        .withSensorToMechanismRatio(IntakeConstants.kIntakeGearRatio))
                .withCurrentLimits(
                    new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(Amps.of(60))
                        .withStatorCurrentLimitEnable(true)
                        .withSupplyCurrentLimit(Amps.of(40))
                        .withSupplyCurrentLimitEnable(true));

        //Apply the configuration to the motor's configurator
        intakeLeaderMotor.getConfigurator().apply(cfg);
        intakeFollowerMotor.getConfigurator().apply(cfg);
        
        //Apply any follower controls
        intakeFollowerMotor.setControl(new Follower(IntakeConstants.kIntakeLeaderID, MotorAlignmentValue.Opposed));
    }

    //Make a method to power the motor
    public void powerIntakeMethod(double power) {
        intakeLeaderMotor.setControl(intakeDutyCycle.withOutput(power));
    }

    //Make a command to power the motor
    public Command powerIntake(double power) {
        return run(
            () -> {
                this.powerIntakeMethod(power);
            }
        );
    }

    //Extra commands
    public Command stopIntake() {
        return powerIntake(0);
    }

}
