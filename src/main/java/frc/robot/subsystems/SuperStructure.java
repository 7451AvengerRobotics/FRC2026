package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.IntakePivot;
import frc.robot.subsystems.Intake.IntakePivot.PivotPosition;
import frc.robot.subsystems.Shooters.Hood;
import frc.robot.subsystems.Shooters.Shooter;

public class SuperStructure {
  private final Intake intake;
  private final Index index;
  private final Feeder feeder;
  private final Shooter shooter;
  private final Hood hood;
  private final IntakePivot pivot;
  private double shooterOffset = 1;

  private boolean passing = false;

  public SuperStructure(
      Index index,
      Intake intake,
      Feeder feeder,
      Shooter shooter,
      Hood hood,
      IntakePivot pivot) {
    this.intake = intake;
    this.index = index;
    this.feeder = feeder;
    this.shooter = shooter;
    this.hood = hood;
    this.pivot = pivot;
  }

  public Command setPassing(boolean passing) {
    return Commands.runOnce(
        () -> {
          this.passing = passing;
        });
  }

  public Command soleIntake() {
    return intake.runIntake(-0.8);
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

  public Command increaseSpeed() {
    return Commands.runOnce(() -> shooterOffset += 0.025);
  }

  public Command decreaseSpeed() {
    return Commands.runOnce(() -> shooterOffset -= 0.025);
  }

  public Command runShooter() {
    return shooter.runShooter(shooterOffset);
  }

  public Command setHoods() {
    return hood.trackHub();
  }

  public Command hoodsUp() {
    return hood.moveUp();
  }

  public Command hoodsDown() {
    return hood.moveDown();
  }

  public Command stopHood() {
    return hood.stop();
  }

  public Command runShooters5000() {
    return shooter.runShooter5000();
  }

  public Command runShooters3000() {
    return shooter.runShooter3000();
  }

  public Command stopShooters() {
    return shooter.stopShooter();
  }

  public Command masterCommand() {
    return Commands.parallel( // These run immediately
        soleIntake(),
        runShooters5000(),
        feeder.runFeeder(-0.9),

        // This branch waits, then starts feeder/index
        Commands.sequence(new WaitCommand(0), Commands.parallel(index.runIndex(-0.9))));
  }

  public Command startupMasterCommand() {
    return Commands.parallel( // These run immediately
        soleIntake(),
        runShooter(),

        // This branch waits, then starts feeder/index
        Commands.sequence(
            new WaitCommand(1.5), Commands.parallel(index.runIndex(-0.6), feeder.runFeeder(-0.9))));
  }

  public Command weirdMasterCommand() {
    return Commands.sequence(
        setPassing(false),
        Commands.parallel(
            soleIntake(), index.runIndex(0.3), feeder.runFeeder(0.2), runShooters5000()));
  }

  public Command strongWeirdMasterCommand() {
    return Commands.sequence(
        setPassing(false),
        Commands.parallel(
            soleIntake(), index.runIndex(0.6), feeder.runFeeder(0.6), runShooters5000()));
  }

  public Command shooterlessMasterCommand() {
    return Commands.parallel(soleIntake(), index.runIndex(-0.9), feeder.runFeeder(-0.9));
  }

  public Command intakelessMasterCommand() {
    return Commands.parallel(
        intake.stopIntake(), index.runIndex(-0.9), feeder.runFeeder(-0.9), runShooter());
  }

  public Command shooterlessWeirdMasterCommand() {
    return Commands.parallel(soleIntake(), index.runIndex(0.6), feeder.runFeeder(-0.6));
  }

  public Command stopMasterCommand() {
    return Commands.parallel(
        intake.stopIntake(), index.stopIndex(), feeder.stopFeeder(), stopShooters());
  }

  public Command deployPivot() {
    return pivot.toPosition(PivotPosition.DEPLOYED.rotations);
  }

  public Command stopPivot() {
    return pivot.stopPivot();
  }

  public Command jiggle() {
    return pivot.jiggle();
  }

  public Command stopJiggle() {
    return deployPivot();
  }

  public Command stowPivot() {
    return pivot.toPosition(0);
  }

  public Command outtake() {
    return Commands.parallel(index.runIndex(0.9), intake.runIntake(1.0));
  }

  public Command resetHoods() {
    return hood.resetHood();
  }

  public Command cut() {
    return Commands.parallel(resetHoods());
  }

  public Command trackHub() {
    return hood.trackHub();
  }
}
