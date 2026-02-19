package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Index extends SubsystemBase {

    private final TalonFX indexMotor;

    public Index() {
        indexMotor = new TalonFX(1); 
    }

    public void runForward() {
        indexMotor.set(0.5); 

    public void runReverse() {
        indexMotor.set(-0.5); 
    }

    public void stop() {
        indexMotor.set(0); 
    }
}
