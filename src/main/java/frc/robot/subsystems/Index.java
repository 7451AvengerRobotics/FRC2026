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
import frc.robot.Constants.IndexConstants;

public class Index extends SubsystemBase {
    //Make the motors
    private final TalonFX indexLeaderMotor = new TalonFX(IndexConstants.kIndexLeaderID);
    private final TalonFX indexFollowerMotor = new TalonFX(IndexConstants.kIndexFollowerID);
    
    //Make the control request
    private final DutyCycleOut indexDutyCycle = new DutyCycleOut(0);

    public Index() {

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
                        .withSensorToMechanismRatio(1))
                .withCurrentLimits(
                    new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(Amps.of(60))
                        .withStatorCurrentLimitEnable(true)
                        .withSupplyCurrentLimit(Amps.of(40))
                        .withSupplyCurrentLimitEnable(true));

        //Apply the configuration to the motor's configurator
        indexLeaderMotor.getConfigurator().apply(cfg);
        indexFollowerMotor.getConfigurator().apply(cfg);
        
        //Apply any follower controls
        indexFollowerMotor.setControl(new Follower(IndexConstants.kIndexLeaderID, MotorAlignmentValue.Opposed));
    }

    //Make a method to power the motor
    public void powerIndexMethod(double power) {
        indexLeaderMotor.setControl(indexDutyCycle.withOutput(power));
    }

    //Make a command to power the motor
    public Command powerIndex(double power) {
        return run(
            () -> {
                this.powerIndexMethod(power);
            }
        );
    }

    //Extra commands
    public Command stopIndex() {
        return powerIndex(0);
    }
    
}
