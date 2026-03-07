package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
    return drive.isRedAlliance();
  }

  public Command depotSideAuto() {
    return Commands.sequence(
        singleAuto(),
        drive.driveToStartingPose1(),
        Commands.parallel(
                superStruc.weirdMasterCommand(),
                drive.followPPPathCommand("DepotSideStartToSource"))
            .withTimeout(4),
        drive.alignToHub(),
        superStruc.masterCommand());
  }

  public Command singleAuto() {
    return Commands.sequence(
        // drive.jostle(),
        superStruc.deployPivot().withTimeout(1),
        drive.alignToHub(),
        superStruc.startupMasterCommand().withTimeout(6));
  }

  public Command middleAuto() {
    return Commands.sequence(
        singleAuto(),
        drive.driveToStartingPose2(),
        Commands.parallel(
                superStruc.weirdMasterCommand(), drive.followPPPathCommand("MiddleStartToSource"))
            .withTimeout(4),
        drive.alignToHub(),
        superStruc.masterCommand());
  }

  public Command sourceSideAuto() {
    return Commands.sequence(
        singleAuto(),
        drive.driveToStartingPose3(),
        Commands.parallel(
                superStruc.weirdMasterCommand(),
                drive.followPPPathCommand("SourceSideStartToSource"))
            .withTimeout(4),
        drive.alignToHub(),
        superStruc.masterCommand());
  }
}
