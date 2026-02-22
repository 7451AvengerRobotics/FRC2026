package frc.robot.subsystems.SimFiles;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TargetConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.drive.Drive;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class TurretSim extends SubsystemBase {
  Drive drive;
  Transform3d turretOffset;
  String name;
  Transform2d turretOffsetTransform2d;
  Pose2d turretPositionPose2d;

  ChassisSpeeds Vr;
  double vxr;
  double vyr;

  private final List<FuelSim> activeFuel = new ArrayList<>();
  private Translation2d target;
  double yf = 1.329;
  double xf;
  double g = 9.8;
  double a = -g;

  public TurretSim(Drive drive, Transform3d turretOffset, String name) {
    this.drive = drive;
    this.turretOffset = turretOffset;
    this.name = name;

    turretOffsetTransform2d =
        new Transform2d(turretOffset.getX(), turretOffset.getY(), new Rotation2d());
    turretPositionPose2d = drive.getPose().plus(turretOffsetTransform2d);

    target = TargetConstants.hub;
  }

  public void setTarget(Translation2d newTarget) {
    this.target = newTarget;
  }

  public Translation2d getTarget() {
    return this.target;
  }

  @Override
  public void periodic() {
    turretPositionPose2d = drive.getPose().plus(turretOffsetTransform2d);
    Vr = drive.getRobotRelativeSpeeds();
    vxr = Vr.vxMetersPerSecond;
    vyr = Vr.vyMetersPerSecond;

    Logger.recordOutput("ZeroedComponentPoses_" + name, new Pose3d[] {new Pose3d()});
    Logger.recordOutput(
        "FinalPoses_" + name,
        new Pose3d[] {new Pose3d(turretOffset.getTranslation(), new Rotation3d(0, 0, calcYaw()))});

    activeFuel.removeIf(
        fuel -> {
          fuel.update(0.02);
          return fuel.isDead();
        });

    Logger.recordOutput(
        "GamePieces/Fuel_" + name,
        activeFuel.stream().map(FuelSim::getPose).toArray(Pose3d[]::new));

    SmartDashboard.putNumber("Yaw", calcYaw());
  }

  public double calcYaw() {
    double deltax = target.getX() - turretPositionPose2d.getX() + vxr * TurretConstants.latency;
    double deltay = target.getY() - turretPositionPose2d.getY() + vyr * TurretConstants.latency;
    double initTheta;

    if (deltax == 0) {
      initTheta = 0;
    } else {
      initTheta = Math.PI - Math.atan(deltay / -deltax);
    }

    double theta = (initTheta - drive.getPose().getRotation().getRadians());

    return mod(theta);
  }

  public double calcYaw(double deltax, double deltay) {
    double initTheta;

    if (deltax == 0) {
      initTheta = 0;
    } else {
      initTheta = Math.PI - Math.atan(deltay / -deltax);
    }

    double theta = (initTheta - drive.getPose().getRotation().getRadians());

    return mod(theta);
  }
  
  public double calcYawForSimBall(double yaw) {
    return yaw + drive.getPose().getRotation().getRadians();
  }

  public double calcVelocity(double xf) {
    double vel = Math.sqrt(g * yf + g * Math.sqrt(Math.pow(xf, 2) + Math.pow(yf, 2))) + 0.5;

    return vel;
  }

  public double calcPitch(double v, double xf) {
    double A = a * Math.pow(xf, 2) / (2 * Math.pow(v, 2));
    double B = xf;
    double C = A - yf;

    double theta = Math.atan((-B - Math.sqrt(Math.pow(B, 2) - 4 * A * C)) / (2 * A));

    return theta;
  }

  public void shootBall() {
    xf =
        Math.sqrt(
            Math.pow((target.getX() - turretPositionPose2d.getX()) + vxr * TurretConstants.latency, 2)
                + Math.pow(
                    (target.getY() - turretPositionPose2d.getY()) + vyr * TurretConstants.latency, 2));

    double v0 = calcVelocity(xf);
    double pitch0 = calcPitch(v0, xf);
    double yaw0 = calcYawForSimBall(calcYaw());

    activeFuel.add(
        new FuelSim(
            v0, pitch0, yaw0, turretPositionPose2d, Vr.vxMetersPerSecond, Vr.vyMetersPerSecond));
  }

  public Command shootBallCommand() {
    return runOnce(
        () -> {
          shootBall();
        });
  }

  // Helper Function:
  public double mod(double angle) {
    return ((angle % (2 * Math.PI)) + (2 * Math.PI)) % (2 * Math.PI);
  }
}
