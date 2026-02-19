package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Index extends SubsystemBase {

    private final TalonFX indexMotor;

    public Index() {
        indexMotor = new TalonFX(1); 
    }


    public void run(double speed) {
        indexMotor.set(speed);
    }

   
    public void stop() {
        indexMotor.set(0);
    }
}
