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
import frc.robot.commands.AutoRoutines;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.IntakePivot;
import frc.robot.subsystems.Shooters.Hood;
import frc.robot.subsystems.Shooters.Shooter;
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
  private final TurretSim simTurret;
  private final Index index = new Index();
  private final Intake intake = new Intake();
  private final IntakePivot pivot = new IntakePivot();
  private final Shooter shooter;
  private final Hood hood;
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
    simTurret = new TurretSim(drive, new Transform3d(-0.17, 0, 0.39, new Rotation3d()));
    shooter = new Shooter(simTurret, drive);
    hood = new Hood(simTurret);
    superStructure = new SuperStructure(index, intake, shooter, hood, pivot);

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

    controller.triangle().onTrue(superStructure.weirdMasterCommand());
    controller.circle().onTrue(superStructure.stopMasterCommand());
    controller.cross().toggleOnTrue(superStructure.masterCommand());
    controller.square().onTrue(superStructure.deployPivot());

    controller.L1().toggleOnTrue(hood.trackHub());
    controller
        .R1()
        .whileTrue(drive.alignToHub(0))
        .onFalse(Commands.run(() -> drive.runVelocity(new ChassisSpeeds(0, 0, 0))));

    controller
        .povLeft()
        .whileTrue(drive.alignForTrench())
        .onFalse(Commands.run(() -> drive.runVelocity(new ChassisSpeeds(0, 0, 0))));
    controller.povDown().whileTrue(drive.moveBackward());
    controller.povUp().whileTrue(drive.moveForward());
    controller.povRight().onTrue(superStructure.resetHoods());

    manip.circle().onTrue(superStructure.stopMasterCommand());
    manip.cross().onTrue(superStructure.masterCommand());
    manip.triangle().onTrue(superStructure.strongWeirdMasterCommand());

    manip.L1().onTrue(superStructure.jiggle()).onFalse(superStructure.stopJiggle());
    manip.R1().onTrue(pivot.runPivot(0));

    manip.povUp().whileTrue(superStructure.hoodsUp()).onFalse(superStructure.stopHood());
    manip.povDown().whileTrue(superStructure.hoodsDown()).onFalse(superStructure.stopHood());
  }

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

    autoChooser.addOption("Hub to Shoot", autos.H_to_S());

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
