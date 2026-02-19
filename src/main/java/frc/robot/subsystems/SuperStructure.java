package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.IntakePivot;

public class SuperStructure {
    private final IntakePivot intakePivot;
    private final Intake intake;
    private final Index index;

    public SuperStructure(Index index, Intake intake, IntakePivot intakePivot) {
        this.intakePivot = intakePivot;
        this.intake = intake;
        this.index = index;
    }

    public Command startIntake() {
        return Commands.parallel(
            intakePivot.toPosition(IntakePivot.PivotPosition.DEPLOYED),
            intake.runIntake(1),
            index.runIndex());
    }
}
