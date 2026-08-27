package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.IntakePivot;
import frc.robot.subsystems.Intake.IntakePivot.PivotPosition;
import frc.robot.subsystems.Shooters.Hood;
import frc.robot.subsystems.Shooters.Shooter;
import frc.robot.subsystems.drive.Drive;

public class SuperStructure {
  private final Drive drive;
  private final Intake intake;
  private final Index index;
  private final Shooter shooter;
  private final Hood hood;
  private final IntakePivot pivot;

  public SuperStructure(
    Drive drive, 
    Index index, 
    Intake intake, 
    Shooter shooter, 
    Hood hood, 
    IntakePivot pivot) {

    this.drive = drive;
    this.intake = intake;
    this.index = index;
    this.shooter = shooter;
    this.hood = hood;
    this.pivot = pivot;
  }

  //Intake Commands
  public Command soleIntake() {
    return intake.runIntake(-0.8);
  }

  public Command stopIntake() {
    return intake.stopIntake();
  }

  public Command reverseIntake() {
    return intake.runIntake(0.8);
  }

  //Index Commands
  public Command soleIndex() {
    return index.runIndex(0.8);
  }

  public Command stopIndex() {
    return index.stopIndex();
  }

  public Command reverseIndex() {
    return index.runIndex(-0.3);
  }

  //Hood Commands
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

  public Command runShooter4000() {
    return shooter.runShooter(4000);
  }

  public Command runShooter3000() {
    return shooter.runShooter(3000);
  }

  public Command stopShooter() {
    return shooter.stopShooter();
  }

  public Command masterCommand() {
    return Commands.parallel( // These run immediately
      soleIntake(),
      soleIndex(),
      runShooter3000());
  }

  public Command weirdMasterCommand() {
    return Commands.parallel(
      soleIntake(), 
      reverseIndex(), 
      runShooter3000());
  }

  public Command shooterlessMasterCommand() {
    return Commands.parallel(
      soleIntake(), 
      soleIndex(), 
      stopShooter());
  }

  public Command intakelessMasterCommand() {
    return Commands.parallel(
      stopIntake(), 
      soleIndex(), 
      runShooter3000());
  }

  public Command shooterlessWeirdMasterCommand() {
    return Commands.parallel(
      soleIntake(), 
      soleIndex(),
      stopShooter());
  }

  public Command stopMasterCommand() {
    return Commands.parallel(
      stopIntake(), 
      stopIndex(), 
      stopShooter(), 
      stopHood());
  }

  public Command botCommand(char intake, char index, char shooter) {
    return Commands.parallel(
      intake == 'F' ? soleIntake() : (intake == 'R' ? reverseIntake() : stopIntake()),
      index == 'F' ? soleIndex() : (index == 'R' ? reverseIndex() : stopIndex()),
      shooter == 'F' ? runShooter3000() : stopShooter());
  }

  public Command deployPivot() {
    return pivot.toPosition(PivotPosition.DEPLOYED.rotations);
  }

  public Command stowPivot() {
    return pivot.toPosition(0);
  }

  public Command stopPivot() {
    return pivot.stopPivot();
  }

  public Command jiggle() {
    return pivot.jiggle();
  }

  public Command resetHood() {
    return hood.resetHood();
  }

  public Command trackHoods() {
    return hood.trackHub();
  }

  public Command reverseOutput() {
    return Commands.parallel(
      reverseIndex(), 
      reverseIntake());
  }

  public Command restingRun() {
    return Commands.parallel(
      stopIntake(), 
      stopIndex(), 
      runShooter3000());
  }
}
