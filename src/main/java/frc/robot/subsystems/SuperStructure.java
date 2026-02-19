package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.IntakePivot;

public class SuperStructure {
    private final IntakePivot intakePivot;

    public SuperStructure(IntakePivot intakePivot) {
        this.intakePivot = intakePivot;
    }

    public Command startIntake() {
        return intakePivot.toPosition(IntakePivot.PivotPosition.DEPLOYED);
    }
}
