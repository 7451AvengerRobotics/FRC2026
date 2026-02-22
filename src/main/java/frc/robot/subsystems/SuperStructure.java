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
        intake.runIntake(1),
        index.runIndex());
  }

  public Command soleIntake() {
    return Commands.run(
        () -> {
          intake.runIntake(1);
        });
  }

  public Command soleIndex() {
    return Commands.run(
        () -> {
          index.runIndex();
        });
  }

  public Command soleFeeder() {
    return Commands.run(
        () -> {
          feeder.runFeeder();
        });
  }

  public Command stopIntake() {
    return Commands.run(
        () -> {
          intake.runIntake(0);
        });
  }

  public Command stopIndex() {
    return Commands.run(
        () -> {
          index.stopIndex();
        });
  }

  public Command stopFeeder() {
    return Commands.run(
        () -> {
          feeder.stopFeeder();
        });
  }

  public Command runShooters() {
    return Commands.parallel(leftShooter.runShooter(), rightShooter.runShooter());
  }

  public Command stopShooters() {
    return Commands.parallel(leftShooter.runShooter(), rightShooter.runShooter());
  }
}
