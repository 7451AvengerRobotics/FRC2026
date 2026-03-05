package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AllianceFlipUtil;

public class AutoRoutines {
  private final Drive drive;
  private final SuperStructure superStruc;

  public AutoRoutines(Drive drive, SuperStructure superStruc) {
    System.out.println("AutoRoutines: Constructing AutoRoutines");
    this.drive = drive;
    this.superStruc = superStruc;
  }

  public Command depotSideAuto() {
    return Commands.sequence(
        drive.driveToPose(AllianceFlipUtil.apply(new Pose2d(3.5, 6.5, new Rotation2d(0)))),
        Commands.parallel(
            superStruc.deployPivot(), drive.followPPPathCommand("DepotSideStartToSource")),
        Commands.waitSeconds(3),
        Commands.parallel(superStruc.startIntake(), drive.followPPPathCommand("SourceToDepot")),
        drive.followPPPathCommand("DepotToShoot"));
  }

  public Command middleAuto() {
    return Commands.sequence(
        drive.driveToPose(AllianceFlipUtil.apply(new Pose2d(3.5, 4, new Rotation2d(0)))),
        Commands.parallel(
            superStruc.deployPivot(), drive.followPPPathCommand("MiddleStartToSource")),
        Commands.waitSeconds(3),
        Commands.parallel(superStruc.startIntake(), drive.followPPPathCommand("SourceToDepot")),
        drive.followPPPathCommand("DepotToShoot"));
  }

  public Command sourceSideAuto() {
    return Commands.sequence(
        drive.driveToPose(AllianceFlipUtil.apply(new Pose2d(3.5, 1.5, new Rotation2d(0)))),
        Commands.parallel(
            superStruc.deployPivot(), drive.followPPPathCommand("SourceSideStartToSource")),
        Commands.waitSeconds(3),
        Commands.parallel(superStruc.startIntake(), drive.followPPPathCommand("SourceToDepot")),
        drive.followPPPathCommand("DepotToShoot"));
  }
}
