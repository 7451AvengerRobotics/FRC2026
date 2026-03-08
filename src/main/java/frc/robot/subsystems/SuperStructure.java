package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.IntakePivot;
import frc.robot.subsystems.Shooters.Hood;
import frc.robot.subsystems.Shooters.Shooter;
import frc.robot.subsystems.Shooters.Turret;

public class SuperStructure {
  private final IntakePivot intakePivot;
  private final Intake intake;
  private final Index index;
  private final Feeder feeder;
  private final Shooter leftShooter;
  private final Shooter rightShooter;
  private final Turret leftTurret;
  private final Turret rightTurret;
  private final Hood leftHood;
  private final Hood rightHood;
  private final IntakePivot pivot;

  public SuperStructure(
      Index index,
      Intake intake,
      IntakePivot intakePivot,
      Feeder feeder,
      Shooter leftShooter,
      Shooter rightShooter,
      Turret leftTurret,
      Turret rightTurret,
      Hood leftHood,
      Hood rightHood,
      IntakePivot pivot) {
    this.intakePivot = intakePivot;
    this.intake = intake;
    this.index = index;
    this.feeder = feeder;
    this.leftShooter = leftShooter;
    this.rightShooter = rightShooter;
    this.leftTurret = leftTurret;
    this.rightTurret = rightTurret;
    this.leftHood = leftHood;
    this.rightHood = rightHood;
    this.pivot = pivot;
  }

  public Command startIntake() {
    return Commands.parallel(
        intakePivot.toPosition(0.27), intake.runIntake(-0.5), index.runIndex(-0.1));
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

  public Command stopTurret() {
    return leftTurret.stopTurret();
  }

  public Command leftShoot() {
    return leftShooter.runShooter();
  }

  public Command rightShoot() {
    return rightShooter.runShooter();
  }

  public Command runShooters() {
    return Commands.parallel(leftShooter.runShooter(), rightShooter.runShooter());
  }

  public Command stopShooters() {
    return Commands.parallel(leftShooter.stopShooter(), rightShooter.stopShooter());
  }

  public Command masterCommand() {
    return Commands.parallel(
        intake.runIntake(-0.5), index.runIndex(-0.9), feeder.runFeeder(-0.9), runShooters());
  }

  public Command weirdMasterCommand() {
    return Commands.parallel(intake.runIntake(-0.75), index.runIndex(0.6), feeder.runFeeder(-0.9));
  }

  public Command shooterlessMasterCommand() {
    return Commands.parallel(
        intake.runIntake(-0.75), index.runIndex(-0.9), feeder.runFeeder(-0.9), runShooters());
  }

  public Command intakelessMasterCommand() {
    return Commands.parallel(
        intake.stopIntake(), index.runIndex(-0.9), feeder.runFeeder(-0.9), runShooters());
  }

  public Command shooterlessWeirdMasterCommand() {
    return Commands.parallel(
        intake.runIntake(-0.75), index.runIndex(0.6), feeder.runFeeder(-0.9), runShooters());
  }

  public Command stopMasterCommand() {
    return Commands.parallel(
        intake.stopIntake(), index.stopIndex(), feeder.stopFeeder(), stopShooters());
  }

  public Command deployPivot() {
    return pivot.toPosition(2.8);
  }

  public Command stowPivot() {
    return pivot.toPosition(0);
  }
}
