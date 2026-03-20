package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.IntakePivot;
import frc.robot.subsystems.Intake.IntakePivot.PivotPosition;
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
  private double shooterOffset = 1;

  private boolean passing = false;

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

  public Command offsetTurrets(double offset) {
    return Commands.run(
        () -> {
          leftTurret.offsetYaw(offset);
          rightTurret.offsetYaw(-offset);
        });
  }

  public Command setPassing(boolean passing) {
    return Commands.runOnce(
        () -> {
          this.passing = passing;
        });
  }

  public Command startIntake() {
    return Commands.parallel(
        intakePivot.toPosition(0.27), intake.runIntake(-0.5), index.runIndex(-0.1));
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

  public Command stopTurret() {
    return leftTurret.stopTurret();
  }

  public Command leftShoot() {
    return leftShooter.runShooter(shooterOffset);
  }

  public Command rightShoot() {
    return rightShooter.runShooter(shooterOffset);
  }

  public Command playThrough() {
    return rightTurret.setTurretPosEncoder(0);
  }

  public Command increaseSpeed() {
    return Commands.runOnce(() -> shooterOffset += 0.025);
  }

  public Command decreaseSpeed() {
    return Commands.runOnce(() -> shooterOffset -= 0.025);
  }

  public Command runShooters() {
    return Commands.parallel(
        leftShooter.runShooter(shooterOffset), rightShooter.runShooter(shooterOffset));
  }

  public Command runShooters(double shooterOffset) {
    return Commands.parallel(
        leftShooter.runShooter(shooterOffset), rightShooter.runShooter(shooterOffset));
  }

  public Command trackTurrets() {
    return Commands.parallel(leftTurret.followHub(), rightTurret.followHub());
  }

  public Command setHoods() {
    return Commands.parallel(leftHood.trackHub(), rightHood.trackHub());
  }

  public Command runShooters5000() {
    return Commands.parallel(leftShooter.runShooter5000(), rightShooter.runShooter5000());
  }

  public Command stopShooters() {
    return Commands.parallel(leftShooter.stopShooter(), rightShooter.stopShooter());
  }

  public Command masterCommand() {
    return Commands.parallel( // These run immediately
        soleIntake(),
        runShooters(),
        feeder.runFeeder(-0.9),

        // This branch waits, then starts feeder/index
        Commands.sequence(new WaitCommand(0), Commands.parallel(index.runIndex(-0.6))));
  }

  public Command startupMasterCommand() {
    return Commands.parallel( // These run immediately
        soleIntake(),
        runShooters(),

        // This branch waits, then starts feeder/index
        Commands.sequence(
            new WaitCommand(1.5), Commands.parallel(index.runIndex(-0.6), feeder.runFeeder(-0.9))));
  }

  public Command weirdMasterCommand() {
    return Commands.sequence(
        setPassing(false),
        Commands.parallel(soleIntake(), index.runIndex(0.6), feeder.runFeeder(0.2), runShooters()));
  }

  public Command shooterlessMasterCommand() {
    return Commands.parallel(soleIntake(), index.runIndex(-0.9), feeder.runFeeder(-0.9));
  }

  public Command intakelessMasterCommand() {
    return Commands.parallel(
        intake.stopIntake(), index.runIndex(-0.9), feeder.runFeeder(-0.9), runShooters());
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

  public Command resetShooters() {
    return Commands.parallel(leftTurret.setTurretPos(2 * Math.PI / 3));
  }

  public Command shootOnMove() {
    return Commands.parallel(trackTurrets(), setHoods(), runShooters());
  }
}
