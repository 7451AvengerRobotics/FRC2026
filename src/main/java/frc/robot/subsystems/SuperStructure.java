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
      Drive drive, Index index, Intake intake, Shooter shooter, Hood hood, IntakePivot pivot) {
    this.drive = drive;
    this.intake = intake;
    this.index = index;
    this.shooter = shooter;
    this.hood = hood;
    this.pivot = pivot;
  }

  public Command soleIntake() {
    return intake.runIntake(-0.8);
  }

  public Command reverseIntake() {
    return intake.runIntake(0.8);
  }

  public Command soleIndex() {
    return index.runIndex(1.0);
  }

  public Command stopIntake() {
    return intake.stopIntake();
  }

  public Command stopIndex() {
    return index.stopIndex();
  }

  public Command reverseIndex() {
    return index.runIndex(-0.3);
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

  public Command runShooters4000() {
    return shooter.runShooter4000();
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
        shooter.setVelCommand(3000),
        hood.trackHub(),
        // This branch waits, then starts feeder/index
        Commands.sequence(new WaitCommand(0), Commands.parallel(index.runIndex(0.8))));
  }

  public Command startupMasterCommand() {
    return Commands.parallel( // These run immediately
        soleIntake(),
        runShooters4000(),

        // This branch waits, then starts feeder/index
        Commands.sequence(new WaitCommand(1.5), Commands.parallel(index.runIndex(-0.6))));
  }

  public Command weirdMasterCommand() {
    return Commands.parallel(soleIntake(), index.runIndex(-0.3), shooter.setVelCommand(3000));
  }

  public Command strongWeirdMasterCommand() {
    return Commands.parallel(soleIntake(), index.runIndex(0.6), runShooters4000());
  }

  public Command shooterlessMasterCommand() {
    return Commands.parallel(soleIntake(), index.runIndex(-0.9), shooter.stopShooter());
  }

  public Command intakelessMasterCommand() {
    return Commands.parallel(intake.stopIntake(), index.runIndex(0.8), runShooters3000());
  }

  public Command shooterlessWeirdMasterCommand() {
    return Commands.parallel(soleIntake(), index.runIndex(0.6));
  }

  public Command stopMasterCommand() {
    return Commands.parallel(intake.stopIntake(), index.stopIndex(), stopShooters(), hood.stop());
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

  public Command resetHoods() {
    return hood.resetHood();
  }

  public Command trackHub() {
    return hood.trackHub();
  }

  public Command intakeBalls() {
    return Commands.parallel(soleIntake(), reverseIndex(), runShooters3000());
  }

  public Command noIntakeBalls() {
    return Commands.parallel(stopIntake(), stopIndex(), runShooters3000());
  }

  public Command shootBalls() {
    return Commands.sequence(
        Commands.parallel(drive.alignToHub(), hood.trackHub()),
        Commands.parallel(soleIndex(), runShooters3000(), soleIntake()));
  }

  public Command noShootBalls() {
    return Commands.sequence(
        Commands.runOnce(() -> drive.runVelocity(new ChassisSpeeds(0, 0, 0))),
        Commands.parallel(stopIndex()));
  }

  public Command restingRun() {
    return Commands.parallel(stopIntake(), stopIndex(), runShooters3000());
  }
}
