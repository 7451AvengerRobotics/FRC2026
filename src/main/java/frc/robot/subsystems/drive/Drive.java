// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.Robot;
import frc.robot.generated.TunerConstants;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.LocalADStarAK;
import java.util.Set;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {
  // TunerConstants doesn't include these constants, so they are declared locally
  static final double ODOMETRY_FREQUENCY = TunerConstants.kCANBus.isNetworkFD() ? 250.0 : 100.0;
  public static final double DRIVE_BASE_RADIUS =
      Math.max(
          Math.max(
              Math.hypot(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
              Math.hypot(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY)),
          Math.max(
              Math.hypot(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
              Math.hypot(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)));

  // PathPlanner config constants
  private static final double ROBOT_MASS_KG = 74.088;
  private static final double ROBOT_MOI = 6.883;
  private static final double WHEEL_COF = 1.2;
  private static final RobotConfig PP_CONFIG =
      new RobotConfig(
          ROBOT_MASS_KG,
          ROBOT_MOI,
          new ModuleConfig(
              TunerConstants.FrontLeft.WheelRadius,
              TunerConstants.kSpeedAt12Volts.in(MetersPerSecond),
              WHEEL_COF,
              DCMotor.getKrakenX60Foc(1)
                  .withReduction(TunerConstants.FrontLeft.DriveMotorGearRatio),
              TunerConstants.FrontLeft.SlipCurrent,
              1),
          getModuleTranslations());

  public static double fieldWidth = Units.feetToMeters(26.0) + Units.inchesToMeters(5.7);
  public static double fieldLength = Units.feetToMeters(54.0) + Units.inchesToMeters(3.2);

  private final PIDController headingController = new PIDController(4, 0.0, 0.0);
  private boolean holonomicControllerActive = false;
  private Pose2d holonomicPoseTarget = new Pose2d();
  private Rotation2d rotation2d = new Rotation2d();
  private final HolonomicDriveWithPIDController holonomicDriveWithPIDController;

  static final Lock odometryLock = new ReentrantLock();
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR
  private final SysIdRoutine sysId;
  private final Alert gyroDisconnectedAlert =
      new Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError);

  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
  private Rotation2d rawGyroRotation = Rotation2d.kZero;
  private SwerveModulePosition[] lastModulePositions = // For delta tracking
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  private SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, Pose2d.kZero);

  public Drive(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO) {
    this.gyroIO = gyroIO;
    modules[0] = new Module(flModuleIO, 0, TunerConstants.FrontLeft);
    modules[1] = new Module(frModuleIO, 1, TunerConstants.FrontRight);
    modules[2] = new Module(blModuleIO, 2, TunerConstants.BackLeft);
    modules[3] = new Module(brModuleIO, 3, TunerConstants.BackRight);
    this.holonomicDriveWithPIDController =
        new HolonomicDriveWithPIDController(
            new PIDController(4, 0, 0),
            new PIDController(4, 0, 0),
            headingController,
            0.5,
            new Pose2d(0.04, 0.04, Rotation2d.fromDegrees(2)),
            1);
    // Usage reporting for swerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);

    // Start odometry thread
    PhoenixOdometryThread.getInstance().start();

    // Configure AutoBuilder for PathPlanner
    AutoBuilder.configure(
        this::getPose,
        this::setPose,
        this::getChassisSpeeds,
        this::runVelocity,
        new PPHolonomicDriveController(
            new PIDConstants(5.0, 0.0, 0.0), new PIDConstants(5.0, 0.0, 0.0)),
        PP_CONFIG,
        () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
        this);
    Pathfinding.setPathfinder(new LocalADStarAK());
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput("Odometry/Trajectory", activePath.toArray(new Pose2d[0]));
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });

    // Configure SysId
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> runCharacterization(voltage.in(Volts)), null, this));
  }

  @Override
  public void periodic() {
    odometryLock.lock(); // Prevents odometry updates while reading data
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);
    for (var module : modules) {
      module.periodic();
    }
    odometryLock.unlock();

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
    }

    // Log empty setpoint states when disabled
    if (DriverStation.isDisabled()) {
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    // Update odometry
    double[] sampleTimestamps =
        modules[0].getOdometryTimestamps(); // All signals are sampled together
    int sampleCount = sampleTimestamps.length;
    for (int i = 0; i < sampleCount; i++) {
      // Read wheel positions and deltas from each module
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
        moduleDeltas[moduleIndex] =
            new SwerveModulePosition(
                modulePositions[moduleIndex].distanceMeters
                    - lastModulePositions[moduleIndex].distanceMeters,
                modulePositions[moduleIndex].angle);
        lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
      }

      // Update gyro angle
      if (gyroInputs.connected) {
        // Use the real gyro angle
        rawGyroRotation = gyroInputs.odometryYawPositions[i];
      } else {
        // Use the angle delta from the kinematics and module deltas
        Twist2d twist = kinematics.toTwist2d(moduleDeltas);
        rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      }

      // Apply update
      poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
    }

    // Update gyro alert
    gyroDisconnectedAlert.set(!gyroInputs.connected && Constants.currentMode != Mode.SIM);
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisSpeeds speeds) {
    // Calculate module setpoints
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, TunerConstants.kSpeedAt12Volts);

    // Log unoptimized setpoints and setpoint speeds
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveChassisSpeeds/Setpoints", discreteSpeeds);

    // Send setpoints to modules
    for (int i = 0; i < 4; i++) {
      modules[i].runSetpoint(setpointStates[i]);
    }

    // Log optimized setpoints (runSetpoint mutates each state)
    Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
  }

  /** Runs the drive in a straight line with the specified drive output. */
  public void runCharacterization(double output) {
    for (int i = 0; i < 4; i++) {
      modules[i].runCharacterization(output);
    }
  }

  /** Stops the drive. */
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = getModuleTranslations()[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0))
        .withTimeout(1.0)
        .andThen(sysId.quasistatic(direction));
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
  }

  /** Returns the module states (turn angles and drive velocities) for all of the modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  /** Returns the module positions (turn angles and drive positions) for all of the modules. */
  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getPosition();
    }
    return states;
  }

  /** Returns the measured chassis speeds of the robot. */
  @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
  private ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  /** Returns the position of each module in radians. */
  public double[] getWheelRadiusCharacterizationPositions() {
    double[] values = new double[4];
    for (int i = 0; i < 4; i++) {
      values[i] = modules[i].getWheelRadiusCharacterizationPosition();
    }
    return values;
  }

  /** Returns the average velocity of the modules in rotations/sec (Phoenix native units). */
  public double getFFCharacterizationVelocity() {
    double output = 0.0;
    for (int i = 0; i < 4; i++) {
      output += modules[i].getFFCharacterizationVelocity() / 4.0;
    }
    return output;
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
  }

  /** Adds a new timestamped vision measurement. */
  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    poseEstimator.addVisionMeasurement(
        visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
  }

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    return TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return getMaxLinearSpeedMetersPerSec() / DRIVE_BASE_RADIUS;
  }

  /** Returns an array of module translations. */
  public static Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
      new Translation2d(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
      new Translation2d(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY),
      new Translation2d(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
      new Translation2d(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)
    };
  }

  public Command followPPPathCommand(String pathName) {
    return Commands.defer(
        () -> {
          try {
            PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

            Pose2d startingPose = path.getStartingHolonomicPose().get();

            return AutoBuilder.resetOdom(startingPose).andThen(AutoBuilder.followPath(path));
          } catch (Exception e) {
            DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
            return Commands.none();
          }
        },
        Set.of(this));
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

  public Command driveToStartingPose1() {
    return Commands.defer(
        () -> {
          return driveToPose(
              new Pose2d(applyX(3.5), applyY(6.5), apply(new Rotation2d(120 / 180 * Math.PI))));
        },
        Set.of(this));
  }

  public Command driveToStartingPose2() {
    return Commands.defer(
        () -> {
          return driveToPose(new Pose2d(applyX(3.5), applyY(4), apply(new Rotation2d(0))));
        },
        Set.of(this));
  }

  public Command driveToStartingPose3() {
    return Commands.defer(
        () -> {
          return driveToPose(new Pose2d(applyX(3.5), applyY(1.5), apply(new Rotation2d(0))));
        },
        Set.of(this));
  }

  public Command driveToPose(Pose2d pose) {
    return Commands.sequence(
            runOnce(
                () -> {
                  holonomicControllerActive = true;
                  holonomicDriveWithPIDController.reset(getPose(), getRobotRelativeSpeeds());
                }),
            run(() -> {
                  this.holonomicPoseTarget = pose;
                  runVelocity(
                      holonomicDriveWithPIDController.calculate(getPose(), holonomicPoseTarget));
                  SmartDashboard.putBoolean(
                      "x controller", holonomicDriveWithPIDController.xReferenceReached());
                  SmartDashboard.putBoolean(
                      "y controller", holonomicDriveWithPIDController.yReferenceReached());
                  SmartDashboard.putBoolean(
                      "rotation controller",
                      holonomicDriveWithPIDController.rotationReferenceReached());
                })
                .until(holonomicDriveWithPIDController::atReference),
            runOnce(this::stop))
        .finallyDo(() -> holonomicControllerActive = false);
  }

  public Command driveToRotation(Rotation2d rotation) {
    return Commands.sequence(
            runOnce(
                () -> {
                  holonomicControllerActive = true;
                  holonomicDriveWithPIDController.reset(getPose(), getRobotRelativeSpeeds());
                }),
            run(() -> {
                  this.rotation2d = rotation;
                  runVelocity(
                      holonomicDriveWithPIDController.calculateRotations(getPose(), rotation2d));
                  SmartDashboard.putBoolean(
                      "x controller", holonomicDriveWithPIDController.xReferenceReached());
                  SmartDashboard.putBoolean(
                      "y controller", holonomicDriveWithPIDController.yReferenceReached());
                  SmartDashboard.putBoolean(
                      "rotation controller",
                      holonomicDriveWithPIDController.rotationReferenceReached());
                })
                .until(holonomicDriveWithPIDController::atReference),
            runOnce(this::stop))
        .finallyDo(() -> holonomicControllerActive = false);
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return kinematics.toChassisSpeeds(
        modules[0].getState(), modules[1].getState(), modules[2].getState(), modules[3].getState());
  }

  public Command jostle() {
    return Commands.defer(
        () -> {
          return Commands.run(
                  () -> {
                    this.runVelocity(new ChassisSpeeds(1, 0, 0));
                  })
              .withTimeout(0.3)
              .andThen(
                  Commands.run(
                      () -> {
                        this.runVelocity(new ChassisSpeeds(0, 0, 0));
                      }));
        },
        Set.of(this));
  }

  public Command alignToHub() {

    return Commands.defer(
        () -> {
          double deltax = applyX(16.54 - 11.915) - getPose().getX();
          double deltay = 4.035 - getPose().getY();

          double initTheta = Math.atan2(deltay, deltax);

          double theta = (initTheta);

          return driveToRotation(new Rotation2d(theta + Math.PI));
        },
        Set.of(this));
  }

  public Command alignForTrench() {
    return Commands.defer(
        () -> {
          boolean flipped = this.getPose().getX() > 4;

          return Commands.run(
              () -> {
                this.driveToPose(
                    new Pose2d(
                        this.getPose().getX(),
                        this.getPose().getY(),
                        this.apply(new Rotation2d(flipped ? Math.PI : 0))));
              });
        },
        Set.of(this));
  }

  public double mod(double angle) {
    return ((angle % (2 * Math.PI)) + (2 * Math.PI)) % (2 * Math.PI);
  }

  public Command driveUnderTrenchToNeutral() {
    if (this.getPose().getY() > 4) {
      return Commands.sequence(
          this.driveToPose(new Pose2d(3.75, 7.375, new Rotation2d())),
          this.followPPPathCommand("DepotSideTrench"));
    } else {
      return Commands.sequence(
          this.driveToPose(new Pose2d(3.75, 0.625, new Rotation2d())),
          this.followPPPathCommand("SourceSideTrench"));
    }
  }

  public Command driveUnderTrenchToAlliance() {
    if (this.getPose().getY() > 4) {
      return Commands.sequence(
          this.driveToPose(new Pose2d(5.25, 7.375, new Rotation2d(Math.PI))),
          this.followPPPathCommand("DepotSideTrenchOpposite"));
    } else {
      return Commands.sequence(
          this.driveToPose(new Pose2d(5.25, 0.625, new Rotation2d(Math.PI))),
          this.followPPPathCommand("SourceSideTrenchOpposite"));
    }
  }

  public Command driveOverBump() {
    if (this.getPose().getY() > 4 && this.getPose().getX() < 4) {
      return Commands.sequence(
          this.driveToPose(AllianceFlipUtil.apply(new Pose2d(2, 5.5, new Rotation2d()))),
          this.followPPPathCommand("DepotSideBump"));
    } else if (this.getPose().getY() < 4 && this.getPose().getX() < 4) {
      return Commands.sequence(
          this.driveToPose(AllianceFlipUtil.apply(new Pose2d(2, 2.5, new Rotation2d()))),
          this.followPPPathCommand("SourceSideBump"));
    } else if (this.getPose().getY() > 4 && this.getPose().getX() > 5.7) {
      return Commands.sequence(
          this.driveToPose(AllianceFlipUtil.apply(new Pose2d(2, 2.5, new Rotation2d()))),
          this.followPPPathCommand("DepotSideBumpOpposite"));
    } else if (this.getPose().getY() < 4 && this.getPose().getX() > 5.7) {
      return Commands.sequence(
          this.driveToPose(AllianceFlipUtil.apply(new Pose2d(2, 2.5, new Rotation2d()))),
          this.followPPPathCommand("SourceSideBumpOpposite"));
    } else {
      return Commands.none();
    }
  }
}
