// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.RobotSide;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.commands.AutoRoutines;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.IntakePivot;
import frc.robot.subsystems.Shooters.Hood;
import frc.robot.subsystems.Shooters.Shooter;
import frc.robot.subsystems.Shooters.Turret;
import frc.robot.subsystems.SimFiles.TurretSim;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.vision.*;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Vision vision;
  private final Turret leftTurret;
  private final Turret rightTurret;
  private final TurretSim simTurretLeft;
  private final TurretSim simTurretRight;
  private final Index index = new Index();
  private final Intake intake = new Intake();
  private final Feeder feeder = new Feeder();
  private final IntakePivot pivot = new IntakePivot();
  private final Shooter leftShooter;
  private final Shooter rightShooter;
  private final Hood leftHood;
  private final Hood rightHood;
  private final SuperStructure superStructure;

  // Controller
  private final CommandPS5Controller controller = new CommandPS5Controller(0);
  private final CommandPS5Controller manip = new CommandPS5Controller(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  private final AutoRoutines autos;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        // ModuleIOTalonFX is intended for modules with TalonFX drive, TalonFX turn, and
        // a CANcoder
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVision(
                    VisionConstants.camera0Name, VisionConstants.robotToCamera0),
                new VisionIOPhotonVision(
                    VisionConstants.camera1Name, VisionConstants.robotToCamera1));

        // The ModuleIOTalonFXS implementation provides an example implementation for
        // TalonFXS controller connected to a CANdi with a PWM encoder. The
        // implementations
        // of ModuleIOTalonFX, ModuleIOTalonFXS, and ModuleIOSpark (from the Spark
        // swerve
        // template) can be freely intermixed to support alternative hardware
        // arrangements.
        // Please see the AdvantageKit template documentation for more information:
        // https://docs.advantagekit.org/getting-started/template-projects/talonfx-swerve-template#custom-module-implementations
        //
        // drive =
        // new Drive(
        // new GyroIOPigeon2(),
        // new ModuleIOTalonFXS(TunerConstants.FrontLeft),
        // new ModuleIOTalonFXS(TunerConstants.FrontRight),
        // new ModuleIOTalonFXS(TunerConstants.BackLeft),
        // new ModuleIOTalonFXS(TunerConstants.BackRight));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(
                    VisionConstants.camera0Name, VisionConstants.robotToCamera0, drive::getPose),
                new VisionIOPhotonVisionSim(
                    VisionConstants.camera1Name, VisionConstants.robotToCamera1, drive::getPose));
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
        break;
    }

    // Other subsystems
    // leftTurret =
    //     new TurretSim(drive, new Transform3d(-0.17, -0.15, 0.39, new Rotation3d()), "Left");
    // rightTurret =
    //     new TurretSim(drive, new Transform3d(-0.17, 0.15, 0.39, new Rotation3d()), "Right");
    simTurretLeft =
        new TurretSim(drive, new Transform3d(-0.17, 0.15, 0.39, new Rotation3d()), "Left");
    simTurretRight =
        new TurretSim(drive, new Transform3d(-0.17, -0.15, 0.39, new Rotation3d()), "Right");

    // simTurret = new TurretSim(drive, new Transform3d(), "Left");

    leftShooter = new Shooter(ShooterConstants.LeftShooterLeaderID, "left", simTurretLeft, drive);
    rightShooter =
        new Shooter(ShooterConstants.RightShooterLeaderID, "right", simTurretRight, drive);

    leftTurret =
        new Turret(
            TurretConstants.kLeftTurretID,
            Constants.RobotSide.LEFT,
            drive,
            new Transform3d(-0.17, 0.15, 0.39, new Rotation3d()),
            simTurretLeft);
    rightTurret =
        new Turret(
            TurretConstants.kRightTurretID,
            Constants.RobotSide.RIGHT,
            drive,
            new Transform3d(-0.17, -0.15, 0.39, new Rotation3d()),
            simTurretRight);

    leftHood =
        new Hood(
            HoodConstants.kLeftHoodMotorID,
            HoodConstants.kLeftHoodEncoderID,
            RobotSide.LEFT,
            simTurretLeft);
    rightHood =
        new Hood(
            HoodConstants.kRightHoodMotorID,
            HoodConstants.kRightHoodEncoderID,
            RobotSide.RIGHT,
            simTurretRight);
    superStructure =
        new SuperStructure(
            index,
            intake,
            feeder,
            leftShooter,
            rightShooter,
            leftTurret,
            rightTurret,
            leftHood,
            rightHood,
            pivot);

    // Set up auto routines
    autos = new AutoRoutines(drive, superStructure);
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Configure the bindings
    configureButtonBindings();
    configureAutos();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    // // Switch to X pattern when X button is pressed
    // controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // controller.L1().onTrue(leftTurret.shootBallCommand());
    // controller.R1().onTrue(rightTurret.shootBallCommand());

    // // Dropping the intake down
    // controller.L2().onTrue(superStructure.startIntake());
    // .toggleOnFalse(superStructure.stopIntake());

    controller.triangle().onTrue(superStructure.weirdMasterCommand());
    controller.circle().onTrue(superStructure.stopMasterCommand());
    controller.cross().toggleOnTrue(superStructure.masterCommand());
    // controller.cross().whileTrue(rightHood.toAngleDegrees(25));

    controller.square().onTrue(superStructure.deployPivot());

    // controller.povUp().toggleOnTrue(superStructure.startupMasterCommand());

    // controller
    //     .touchpad()
    //     .onTrue(
    //         Commands.parallel(
    //             leftTurret.pass(), rightTurret.pass(), leftHood.pass(), rightHood.pass()));

    // controller.L1().onTrue(superStructure.masterCommand());
    // controller.R1().onTrue(superStructure.stopMasterCommand());

    // controller.L1().onTrue(simTurretLeft.shootBallCommand());
    controller
        .L1()
        .onTrue(drive.alignToHub(0));

    // controller.R1().onTrue(Commands.parallel(rightTurret.followHub(), leftTurret.followHub()));

    controller
        .povLeft()
        .whileTrue(drive.alignForTrench())
        .onFalse(Commands.run(() -> drive.runVelocity(new ChassisSpeeds(0, 0, 0))));
    manip.povDown().whileTrue(drive.moveBackward());
    manip.povUp().whileTrue(drive.moveForward());

    // controller
    //     .square()
    //     .onTrue(
    //         Commands.parallel(
    //             simTurretLeft.setTargetCommand(TargetConstants.hub),
    //             simTurretRight.setTargetCommand((TargetConstants.hub))));
    // controller
    //     .triangle()
    //     .onTrue(
    //         Commands.parallel(
    //             simTurretLeft.setTargetCommand(TargetConstants.hub),
    //             simTurretRight.setTargetCommand((TargetConstants.hub))));
    // controller
    //     .circle()
    //     .onTrue(
    //         Commands.parallel(
    //             simTurretLeft.setTargetCommand(TargetConstants.hub),
    //             simTurretRight.setTargetCommand((TargetConstants.hub))));
    // controller
    //     .cross()
    //     .onTrue(
    //         Commands.parallel(
    //             simTurretLeft.setTargetCommand(TargetConstants.hub),
    //             simTurretRight.setTargetCommand((TargetConstants.hub))));

    // controller.touchpad().onTrue(drive.driveOverBump());

    // manip.PS().onTrue(superStructure.stopIntake());

    // manip.povLeft().onTrue(superStructure.offsetTurrets(-5 * Math.PI / 180));
    // manip.povRight().onTrue(superStructure.offsetTurrets(5 * Math.PI / 180));

    // manip
    //    .L1()
    //    .whileTrue(Commands.parallel(superStructure.jiggle(), superStructure.stopIntake()))
    //    .onFalse(Commands.parallel(superStructure.stopJiggle()));
    // // manip.R1().onTrue(superStructure.stopPivot());
    // manip
    //     .povUp()
    //     .onTrue(superStructure.increaseSpeed())
    //     .toggleOnTrue(superStructure.runShooters(1.1));
    // manip
    //     .povDown()
    //     .onTrue(superStructure.decreaseSpeed())
    //     .toggleOnTrue(superStructure.runShooters(0.9));
    manip.povUp().whileTrue(superStructure.hoodsUp()).onFalse(superStructure.stopHoods());
    manip.povDown().whileTrue(superStructure.hoodsDown()).onFalse(superStructure.stopHoods());
    // manip
    //     .povLeft()
    //     .whileTrue(
    //         Commands.parallel(
    //             leftTurret.alignWithOffsetAngle(-5), rightTurret.alignWithOffsetAngle(-5)));
    // manip
    //     .povRight()
    //     .whileTrue(
    //         Commands.parallel(
    //             leftTurret.alignWithOffsetAngle(5), rightTurret.alignWithOffsetAngle(5)));

    // manip
    //     .povLeft()
    //     .onTrue(superStructure.increaseSpeed())
    //     .toggleOnTrue(superStructure.runShooters(0.95));
    // manip
    //     .povRight()
    //     .onTrue(superStructure.decreaseSpeed())
    //     .toggleOnTrue(superStructure.runShooters(1.05));
    // manip.square().onTrue(drive.alignToHub(-5));
    // manip.circle().onTrue(drive.alignToHub(5));

    manip.circle().onTrue(superStructure.stopMasterCommand());
    manip.cross().onTrue(superStructure.masterCommand());
    manip.triangle().onTrue(superStructure.strongWeirdMasterCommand());
    // controller.L1().onTrue(rightTurret.disableTurret());
    manip.L1().onTrue(superStructure.jiggle()).onFalse(superStructure.stopJiggle());
    manip.R1().onTrue(pivot.runPivot(0));
    // manip
    //     .touchpad()
    //     .onTrue(
    //         Commands.parallel(
    //             leftTurret.setTurretPosEncoder(2.5), rightTurret.setTurretPosEncoder(2.5)));

    // manip.triangle().whileTrue(superStructure.reverseIntake());

    // manip.R1().onTrue(superStructure.deployPivot());
    // manip.R1().onTrue(superStructure.offsetShooters(0.025));

    // controller.PS().onTrue(superStructure.stopShooters());

    // controller.touchpad().toggleOnTrue(superStructure.playThrough());
  }

  // public Command driveOverSourceSideBump(){
  //   return Commands.sequence(Commands.run(() -> {
  //     drive.driveToPose(new Pose2d(0, 0, null))
  //   }, null))
  // }

  /*
  > Task :discoverroborio
  Discovering Target roborio
  admin @ 10.74.51.2: Resolved but not connected.
    Reason: TimeoutException
    Discovery timed out.
  admin @ 172.22.11.2: Resolved but not connected.
    Reason: TimeoutException
    Discovery timed out.
  admin @ roborio-7451-FRC.lan: Resolved but not connected.
    Reason: TimeoutException
    Discovery timed out.
  admin @ roborio-7451-FRC: Resolved but not connected.
    Reason: TimeoutException
    Discovery timed out.
  admin @ roborio-7451-FRC.local: Resolved but not connected.
    Reason: TimeoutException
    Discovery timed out.
  admin @ null: Resolved but not connected.
    Reason: TimeoutException
    Discovery timed out.
  admin @ roborio-7451-FRC.frc-field.local: Failed resolution.
    Reason: RuntimeException
    Unknown Host */

  public void configureAutos() {
    // AdvantageKit autos
    // autoChooser.addOption(
    //     "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    // autoChooser.addOption(
    //     "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    // autoChooser.addOption(
    //     "Drive SysId (Quasistatic Forward)",
    //     drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Drive SysId (Quasistatic Reverse)",
    //     drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // autoChooser.addOption(
    //     "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Depot Side
    autoChooser.addOption("DF", autos.DF());
    autoChooser.addOption("DN", autos.DN());
    autoChooser.addOption("DF_D2", autos.DF_D2());
    autoChooser.addOption("DN_D2", autos.DN_D2());

    // Swerve Align Depot
    autoChooser.addOption("DF_Sw", autos.DF_Sw());
    autoChooser.addOption("DN_Sw", autos.DN_Sw());
    autoChooser.addOption("DF_D2_Sw", autos.DF_D2_Sw());
    autoChooser.addOption("DN_D2_Sw", autos.DN_D2_Sw());

    // Source Side
    autoChooser.addOption("SF", autos.SF());
    autoChooser.addOption("SN", autos.SN());
    autoChooser.addOption("SF_S2", autos.SF_S2());
    autoChooser.addOption("SN_S2", autos.SN_S2());

    // Swerve Align Source
    autoChooser.addOption("SF_Sw", autos.SF_Sw());
    autoChooser.addOption("SN_Sw", autos.SN_Sw());
    autoChooser.addOption("SF_S2_Sw", autos.SF_S2_Sw());
    autoChooser.addOption("SN_S2_Sw", autos.SN_S2_Sw());

    // Bump Start
    autoChooser.addOption("DepotDepot", autos.depotDepot());
    autoChooser.addOption("DepotSource", autos.depotSource());
    autoChooser.addOption("SourceDepot", autos.sourceDepot());
    autoChooser.addOption("SourceSource", autos.sourceSource());

    // Preload
    autoChooser.addOption("Single Auto", autos.singleAuto());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
