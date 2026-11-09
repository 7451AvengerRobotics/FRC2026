package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {

    //Make the motors
    public final TalonFX shooterLeader = new TalonFX(ShooterConstants.kShooterLeaderID);
    public final TalonFX shooterFollower = new TalonFX(ShooterConstants.kShooterFollowerID);

    //Make the control request
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0);

    public Shooter() {
        
        //Make a configuration (settings) for your motor
        TalonFXConfiguration cfg =
            new TalonFXConfiguration()
                .withMotorOutput(
                    new MotorOutputConfigs()
                        .withInverted(InvertedValue.CounterClockwise_Positive)
                        .withNeutralMode(NeutralModeValue.Coast))
                .withCurrentLimits(
                    new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(Amps.of(60))
                        .withStatorCurrentLimitEnable(true)
                        .withSupplyCurrentLimit(Amps.of(45))
                        .withSupplyCurrentLimitEnable(true))
                .withSlot0(
                    new Slot0Configs()
                        .withKP(ShooterConstants.kP)
                        .withKI(ShooterConstants.kI)
                        .withKD(ShooterConstants.kD)
                        .withKS(ShooterConstants.kS)
                        .withKV(ShooterConstants.kV)
                        .withKA(ShooterConstants.kA));

        //Apply the configuration to the motor's configurator               
        shooterLeader.getConfigurator().apply(cfg);
        shooterFollower.getConfigurator().apply(cfg);

        //Apply any follower controls
        shooterFollower.setControl(
            new Follower(ShooterConstants.kShooterLeaderID, MotorAlignmentValue.Opposed));
    }

    //Make a method to power the motor
    public void setVel(double rpm) {
        shooterLeader.setControl(velocityRequest.withVelocity(rpm / 60));
    }

    //Make a command to power the motor
    public Command spinShooter(double rpm) {
        return run(
            () -> {
                this.setVel(rpm);
            });
    }

    //Extra commands
    public Command stopShooter() {
        return spinShooter(0);
    }

    public double getCurrVel() {
        return shooterLeader.getVelocity().getValueAsDouble() * 60;
    }
}
