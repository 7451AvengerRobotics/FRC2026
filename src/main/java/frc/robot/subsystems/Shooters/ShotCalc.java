package frc.robot.subsystems.Shooters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;
import frc.robot.Constants.TargetConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.drive.Drive;

public class ShotCalc {
  private double g = 9.81;
  private double xf;
  private double yf;
  private ChassisSpeeds vr;
  private double vxr;
  private double vyr;
  private double pitch = Math.toRadians(55);
  private Transform3d turretOffset = new Transform3d(-0.17, -0.15, 0.39, new Rotation3d());
  private Drive drive;
  private Translation2d target;

  public ShotCalc(Drive drive) {
    this.drive = drive;
  }

  public void setTarget(Translation2d newTarget) {
    target = newTarget;
  }

  public void setCurrState() {
    turretPositionPose2d = drive.getPose().plus();
    vr = ChassisSpeeds.fromRobotRelativeSpeeds(drive.getRobotRelativeSpeeds(), drive.getPose().getRotation());
    vxr = vr.vxMetersPerSecond;
    vyr = vr.vyMetersPerSecond;

    xf =
        Math.sqrt(
            Math.pow(
                    (target.getX() - turretPositionPose2d.getX()) + vxr * TurretConstants.latency,
                    2)
                + Math.pow(
                    (target.getY() - turretPositionPose2d.getY()) + vyr * TurretConstants.latency,
                    2));
  }

  public double getVelocity(double xf) {
    double numerator = g*Math.pow(xf, 2);
    double denom1 = -yf + xf * Math.tan(pitch);
    double denom2 = 2 * Math.pow(Math.cos(pitch), 2);
    return Math.sqrt(numerator / (denom1 * denom2));
    // return Math.sqrt(
    //     (-g * Math.pow(xf, 2)) / (2 * Math.pow(Math.cos(pitch), 2) * (yf - Math.tan(pitch) * xf)));
  }

  public double getYaw(Pose2d robotPose) {
    Transform2d turretOffsetTransform2d =
        new Transform2d(turretOffset.getX(), turretOffset.getY(), new Rotation2d());
    Pose2d turretPositionPose2d = robotPose.plus(turretOffsetTransform2d);

    double deltax = Constants.TargetConstants.hub.getX() - turretPositionPose2d.getX();
    double deltay = Constants.TargetConstants.hub.getY() - turretPositionPose2d.getY();

    double initTheta = Math.PI - Math.atan2(deltay, -deltax);

    double theta = (initTheta - robotPose.getRotation().getRadians());

    return mod(theta);
  }

  public double getTime(double xf) {
    return xf / (getVelocity(xf) * Math.cos(pitch));
  }

  public double getMovingVelocity(double xf, ChassisSpeeds Vr, Pose2d robotPose) {
    double vxri = Vr.vxMetersPerSecond;
    double vyri = Vr.vyMetersPerSecond;

    double robotVelocityMag = Math.sqrt(Math.pow(vxri, 2) + Math.pow(vyri, 2));

    double robotVelocityAngle = Math.atan2(vyri, vxri);

    double transformedVelocityAngle = robotVelocityAngle - getYaw(robotPose) + Math.PI / 2;

    double vrxf = Math.cos(transformedVelocityAngle) * robotVelocityMag;

    double vrxHorizontalDisplacement = -vrxf * getTime(xf);

    double newxf = Math.sqrt(Math.pow(xf, 2) + Math.pow(vrxHorizontalDisplacement, 2));

    return getVelocity(newxf);
  }

  public double getMovingYaw(double xf, ChassisSpeeds Vr, Pose2d robotPose) {
    double vxri = Vr.vxMetersPerSecond;
    double vyri = Vr.vyMetersPerSecond;

    double robotVelocityMag = Math.sqrt(Math.pow(vxri, 2) + Math.pow(vyri, 2));

    double robotVelocityAngle = Math.atan2(vyri, vxri);

    double transformedVelocityAngle = robotVelocityAngle - getYaw(robotPose) + Math.PI / 2;

    double vrxf = Math.cos(transformedVelocityAngle) * robotVelocityMag;

    double vrxHorizontalDisplacement = -vrxf * getTime(xf);

    double yawAdj = Math.atan2(vrxHorizontalDisplacement, xf);

    return mod(getYaw(robotPose) + yawAdj);
  }

  public double mod(double angle) {
    return ((angle % (2 * Math.PI)) + (2 * Math.PI)) % (2 * Math.PI);
  }
}
