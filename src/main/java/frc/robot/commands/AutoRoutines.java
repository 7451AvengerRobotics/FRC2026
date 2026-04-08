package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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

  // Depot Side Autons
  public Command DF() {
    return Commands.sequence(
        Commands.deadline(
            drive.followPPPathCommand("DT-DNZ-F").withTimeout(5),
            Commands.sequence(
                superStruc.deployPivot().withTimeout(1),
                superStruc.stopPivot().withTimeout(0.1),
                superStruc.weirdMasterCommand())),
        drive.driveToDepotReturn().withTimeout(1),
        drive.followPPPathCommand("DNZ-DB").withTimeout(3),
        score());
  }

  public Command D2() {
    return Commands.sequence(
        drive.driveToDX2Start().withTimeout(1),
        Commands.deadline(
            drive.followPPPathCommand("DT-DNZ-2").withTimeout(12), superStruc.weirdMasterCommand()),
        score());
  }

  public Command DF_D2() {
    return Commands.sequence(DF().withTimeout(9), D2());
  }

  public Command DN() {
    return Commands.sequence(
        Commands.deadline(
            drive.followPPPathCommand("DT-DNZ-N").withTimeout(5),
            Commands.sequence(
                superStruc.deployPivot().withTimeout(1),
                superStruc.stopPivot().withTimeout(0.1),
                superStruc.weirdMasterCommand())),
        drive.driveToDepotReturn().withTimeout(1),
        drive.followPPPathCommand("DNZ-DB").withTimeout(3),
        score());
  }

  public Command DN_D2() {
    return Commands.sequence(DN().withTimeout(10), D2());
  }

  // Sweve Align Auto
  public Command DF_Sw() {
    return Commands.sequence(
        Commands.deadline(
            drive.followPPPathCommand("DT-DNZ-F").withTimeout(5),
            Commands.sequence(
                superStruc.deployPivot().withTimeout(1),
                superStruc.stopPivot().withTimeout(0.1),
                superStruc.weirdMasterCommand())),
        drive.driveToDepotReturn().withTimeout(1),
        drive.followPPPathCommand("DNZ-DB").withTimeout(3),
        score());
  }

  public Command D2_Sw() {
    return Commands.sequence(
        drive.driveToDX2Start().withTimeout(1),
        Commands.deadline(
            drive.followPPPathCommand("DT-DNZ-2").withTimeout(12), superStruc.weirdMasterCommand()),
        score());
  }

  public Command DN_Sw() {
    return Commands.sequence(
        Commands.deadline(
            drive.followPPPathCommand("DT-DNZ-N").withTimeout(5),
            Commands.sequence(
                superStruc.deployPivot().withTimeout(1),
                superStruc.stopPivot().withTimeout(0.1),
                superStruc.weirdMasterCommand())),
        drive.driveToDepotReturn().withTimeout(1),
        drive.followPPPathCommand("DNZ-DB").withTimeout(3),
        score());
  }

  public Command DF_D2_Sw() {
    return Commands.sequence(DF_Sw().withTimeout(9), D2_Sw());
  }

  public Command DN_D2_Sw() {
    return Commands.sequence(DN_Sw().withTimeout(10), D2_Sw());
  }

  // Source Side Autons
  public Command SF() {
    return Commands.sequence(
        Commands.deadline(
            drive.followPPPathCommand("DT-DNZ-F").withTimeout(5),
            Commands.sequence(
                superStruc.deployPivot().withTimeout(1),
                superStruc.stopPivot().withTimeout(0.1),
                superStruc.weirdMasterCommand())),
        drive.driveToDepotReturn().withTimeout(1),
        drive.followPPPathCommand("DNZ-DB").withTimeout(3),
        score());
  }

  public Command S2() {
    return Commands.sequence(
        drive.driveToDX2Start().withTimeout(1),
        Commands.deadline(
            drive.followPPPathCommand("DT-DNZ-2").withTimeout(12), superStruc.weirdMasterCommand()),
        score());
  }

  public Command SF_S2() {
    return Commands.sequence(SF().withTimeout(9), S2());
  }

  public Command SN() {
    return Commands.sequence(
        Commands.deadline(
            drive.followPPPathCommand("ST-SNZ-N").withTimeout(5),
            Commands.sequence(
                superStruc.deployPivot().withTimeout(1),
                superStruc.stopPivot().withTimeout(0.1),
                superStruc.weirdMasterCommand())),
        drive.driveToDepotReturn().withTimeout(1),
        drive.followPPPathCommand("SNZ-SB").withTimeout(3),
        score());
  }

  public Command SN_S2() {
    return Commands.sequence(SN().withTimeout(10), S2());
  }

  // Sweve Align Auto
  public Command SF_Sw() {
    return Commands.sequence(
        Commands.deadline(
            drive.followPPPathCommand("ST-SNZ-F").withTimeout(5),
            Commands.sequence(
                superStruc.deployPivot().withTimeout(1),
                superStruc.stopPivot().withTimeout(0.1),
                superStruc.weirdMasterCommand())),
        drive.driveToDepotReturn().withTimeout(1),
        drive.followPPPathCommand("SNZ-SB").withTimeout(3),
        score());
  }

  public Command S2_Sw() {
    return Commands.sequence(
        drive.driveToDX2Start().withTimeout(1),
        Commands.deadline(
            drive.followPPPathCommand("ST-SNZ-2").withTimeout(12), superStruc.weirdMasterCommand()),
        score());
  }

  public Command SF_S2_Sw() {
    return Commands.sequence(SF_Sw().withTimeout(9), S2_Sw());
  }

  public Command SN_Sw() {
    return Commands.sequence(
        Commands.deadline(
            drive.followPPPathCommand("ST-SNZ-N").withTimeout(5),
            Commands.sequence(
                superStruc.deployPivot().withTimeout(1),
                superStruc.stopPivot().withTimeout(0.1),
                superStruc.weirdMasterCommand())),
        drive.driveToDepotReturn().withTimeout(1),
        drive.followPPPathCommand("SNZ-SB").withTimeout(3),
        score());
  }

  public Command SN_S2_Sw() {
    return Commands.sequence(SN_Sw().withTimeout(10), S2_Sw());
  }

  // Bump Start Autons
  public Command depotDepot() {
    return Commands.sequence(
        Commands.parallel(superStruc.deployPivot()).withTimeout(2),
        Commands.deadline(
            drive.followPPPathCommand("DB-DNZ").withTimeout(5),
            Commands.sequence(
                superStruc.stopPivot().withTimeout(1), superStruc.weirdMasterCommand())),
        Commands.deadline(
            drive.driveToDepotReturn().withTimeout(2), superStruc.weirdMasterCommand()),
        Commands.deadline(
            drive.followPPPathCommand("DNZ-DB").withTimeout(4), superStruc.weirdMasterCommand()),
        score());
  }

  public Command depotSource() {
    return Commands.sequence(
        Commands.race(drive.driveToStartDB(), superStruc.deployPivot()),
        Commands.deadline(
            drive.followPPPathCommand("DB-DNZ"),
            Commands.sequence(
                superStruc.deployPivot().withTimeout(1), superStruc.weirdMasterCommand())),
        Commands.deadline(drive.driveToDepotReturn(), superStruc.weirdMasterCommand()),
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
            Commands.deadline(drive.driveToSourceReturn()), superStruc.weirdMasterCommand()),
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
        Commands.deadline(drive.driveToSourceReturn(), superStruc.weirdMasterCommand()),
        Commands.deadline(drive.followPPPathCommand("SNZ-DB"), superStruc.weirdMasterCommand()),
        score());
  }

  public Command scoreWithDriveAlign() {
    return Commands.sequence(
        drive.alignToHub().withTimeout(2),
        Commands.run(() -> drive.runVelocity(new ChassisSpeeds(0, 0, 0))).withTimeout(0.5),
        Commands.parallel(superStruc.masterCommand()));
  }

  public Command score() {
    return Commands.parallel(
        Commands.run(() -> drive.runVelocity(new ChassisSpeeds(0, 0, 0))).withTimeout(0.5),
        superStruc.masterCommand());
  }

  public Command singleAuto() {
    return Commands.sequence(
        // drive.jostle(),
        superStruc.deployPivot().withTimeout(1),
        drive.alignToHub().withTimeout(2),
        Commands.run(() -> drive.runVelocity(new ChassisSpeeds(0, 0, 0))).withTimeout(0.5),
        superStruc.startupMasterCommand().withTimeout(6));
  }
}
