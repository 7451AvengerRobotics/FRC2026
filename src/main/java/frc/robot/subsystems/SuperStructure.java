package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.IntakePivot;
import frc.robot.subsystems.Shooters.Shooter;

public class SuperStructure {
  private final IntakePivot intakePivot;
  private final Intake intake;
  private final Index index;
  private final Feeder feeder;
  private final Shooter leftShooter;
  private final Shooter rightShooter;

  public SuperStructure(
      Index index,
      Intake intake,
      IntakePivot intakePivot,
      Feeder feeder,
      Shooter leftShooter,
      Shooter rightShooter) {
    this.intakePivot = intakePivot;
    this.intake = intake;
    this.index = index;
    this.feeder = feeder;
    this.leftShooter = leftShooter;
    this.rightShooter = rightShooter;
  }

  public Command startIntake() {
    return Commands.parallel(
        intakePivot.toPosition(IntakePivot.PivotPosition.DEPLOYED),
        intake.runIntake(-0.5),
        index.runIndex(-0.1));
  }

  public Command soleIntake() {
    return intake.runIntake(-0.5);
  }

  public Command soleIndex() {
    return index.runIndex(-0.5);
  }

  public Command soleFeeder() {
    return feeder.runFeeder(-0.3);
  }

  public Command stopIntake() {
    return intake.stopIntake();
  }

  public Command stopIndex() {
    return index.stopIndex();
  }

  public Command stopFeeder() {
    return feeder.stopFeeder();
  }

  public Command reverseIndex() {
    return index.runIndex(0.3);
  }

  public Command runShooters() {
    return Commands.parallel(leftShooter.setVelCommand(4400), rightShooter.setVelCommand(4400));
  }

  public Command stopShooters() {
    return Commands.parallel(leftShooter.stopShooter(), rightShooter.stopShooter());
  }

  public Command masterCommand() {
    return Commands.parallel(
        intake.runIntake(-0.75), index.runIndex(-0.9), feeder.runFeeder(-0.9), runShooters());
  }

  public Command weirdMasterCommand() {
    return Commands.parallel(
        intake.runIntake(-0.75), index.runIndex(0.6), feeder.runFeeder(-0.9), runShooters());
  }

  public Command stopMasterCommand() {
    return Commands.parallel(
        intake.stopIntake(), index.stopIndex(), feeder.stopFeeder(), stopShooters());
  }
}
