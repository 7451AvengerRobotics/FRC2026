package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.drive.Drive;

public class AutoRoutines {
  private final Drive drive;
  private final SuperStructure superStruc;

  public static double fieldWidth = Units.feetToMeters(26.0) + Units.inchesToMeters(5.0);
  public static double fieldLength = Units.feetToMeters(57.0) + Units.inchesToMeters(6.875);

  public AutoRoutines(Drive drive, SuperStructure superStruc) {
    System.out.println("AutoRoutines: Constructing AutoRoutines");
    this.drive = drive;
    this.superStruc = superStruc;
  }

  public double applyX(double x) {
    return shouldFlip() ? fieldLength - x : x;
  }

  public double applyY(double y) {
    return shouldFlip() ? fieldWidth - y : y;
  }

  public Translation2d apply(Translation2d translation) {
    return new Translation2d(applyX(translation.getX()), applyY(translation.getY()));
  }

  public Rotation2d apply(Rotation2d rotation) {
    return shouldFlip() ? rotation.rotateBy(Rotation2d.kPi) : rotation;
  }

  public Pose2d apply(Pose2d pose) {
    return shouldFlip()
        ? new Pose2d(apply(pose.getTranslation()), apply(pose.getRotation()))
        : pose;
  }

  public boolean shouldFlip() {
    return Robot.IsRedAlliance.getAsBoolean();
  }

  public Command depotDepot() {
    return Commands.sequence(
        Commands.race(drive.driveToStartDB(), superStruc.deployPivot()),
        Commands.deadline(
            drive.followPPPathCommand("DB-DNZ"),
            Commands.sequence(
                superStruc.deployPivot().withTimeout(1), superStruc.weirdMasterCommand())),
        Commands.deadline(
            Commands.deadline(drive.driveToDSReturn()), superStruc.weirdMasterCommand()),
        Commands.deadline(drive.followPPPathCommand("DNZ-DB"), superStruc.weirdMasterCommand()),
        score());
  }

  public Command depotSource() {
    return Commands.sequence(
        Commands.race(drive.driveToStartDB(), superStruc.deployPivot()),
        Commands.deadline(
            drive.followPPPathCommand("DB-DNZ"),
            Commands.sequence(
                superStruc.deployPivot().withTimeout(1), superStruc.weirdMasterCommand())),
        Commands.deadline(drive.driveToDSReturn(), superStruc.weirdMasterCommand()),
        Commands.deadline(drive.followPPPathCommand("DNZ-SB"), superStruc.weirdMasterCommand()),
        score());
  }

  public Command sourceSource() {
    return Commands.sequence(
        Commands.race(drive.driveToStartSB(), superStruc.deployPivot()),
        Commands.deadline(
            drive.followPPPathCommand("SB-SNZ"),
            Commands.sequence(
                superStruc.deployPivot().withTimeout(1), superStruc.weirdMasterCommand())),
        Commands.deadline(
            Commands.deadline(drive.driveToSSReturn()), superStruc.weirdMasterCommand()),
        Commands.deadline(drive.followPPPathCommand("SNZ-SB"), superStruc.weirdMasterCommand()),
        score());
  }

  public Command sourceDepot() {
    return Commands.sequence(
        Commands.race(drive.driveToStartSB(), superStruc.deployPivot()),
        Commands.deadline(
            drive.followPPPathCommand("SB-SNZ"),
            Commands.sequence(
                superStruc.deployPivot().withTimeout(1), superStruc.weirdMasterCommand())),
        Commands.deadline(drive.driveToSSReturn(), superStruc.weirdMasterCommand()),
        Commands.deadline(drive.followPPPathCommand("SNZDB"), superStruc.weirdMasterCommand()),
        score());
  }

  public Command score() {
    return Commands.sequence(
        drive.alignToHub().withTimeout(0.5),
        drive.alignToHub().withTimeout(3),
        superStruc.masterCommand());
  }

  public Command singleAuto() {
    return Commands.sequence(
        // drive.jostle(),
        superStruc.deployPivot().withTimeout(1),
        drive.alignToHub(),
        superStruc.startupMasterCommand().withTimeout(6));
  }
}
