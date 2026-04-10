package frc.robot.subsystems.Shooters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;

public class ShotCalc {
  double g = 9.81;
  double yf = Constants.TargetConstants.yf;
  double H = 2.5;
  double shotOffset = -0.35;
  public double pitch = Math.toRadians(60);
  Transform3d turretOffset = new Transform3d();
  InterpolatingDoubleTreeMap angleLerp = new InterpolatingDoubleTreeMap();
  InterpolatingDoubleTreeMap velocityLerp = new InterpolatingDoubleTreeMap();

  public ShotCalc(Transform3d turretOffset) {
    this.turretOffset = turretOffset;
    angleLerp.put(1.0687 + shotOffset, 0.2548162392); // 14.6°
    angleLerp.put(1.42 + shotOffset, 0.2715680592); // 15.56°
    angleLerp.put(1.87 + shotOffset, 0.2858797963); // 16.38°
    angleLerp.put(2.24 + shotOffset, 0.3053273467); // 17.5°
    angleLerp.put(2.465 + shotOffset, 0.3249698457); // 18.62°
    angleLerp.put(2.7 + shotOffset, 0.3403288802); // 19.5°
    angleLerp.put(3.03 + shotOffset, 0.3589263261); // 20.56°
    angleLerp.put(3.15 + shotOffset, 0.3938758705); // 22.56°
    angleLerp.put(3.3 + shotOffset, 0.4099923863); // 23.48°
    angleLerp.put(3.45 + shotOffset, 0.4257852268); // 24.39°
    angleLerp.put(3.65 + shotOffset, 0.4485343888); // 25.7°
    angleLerp.put(3.84 + shotOffset, 0.4669553025); // 26.75°
    angleLerp.put(3.97 + shotOffset, 0.4839191944); // 27.727°
    angleLerp.put(4.07 + shotOffset, 0.5028848730); // 28.81°
    angleLerp.put(4.15 + shotOffset, 0.5152020140); // 29.52°
    angleLerp.put(4.32 + shotOffset, 0.5327448563); // 30.53°
    angleLerp.put(4.34 + shotOffset, 0.5501223593); // 31.52°
    angleLerp.put(4.41 + shotOffset, 0.5860784353); // 33.57°
    angleLerp.put(4.56 + shotOffset, 0.6083099749); // 34.85°
    angleLerp.put(4.69 + shotOffset, 0.6592155526); // 37.76°
    angleLerp.put(4.63 + shotOffset, 0.6835955730); // 39.16°
    angleLerp.put(4.83 + shotOffset, 0.7133274470); // 40.87°
    angleLerp.put(4.69 + shotOffset, 0.7436549058); // 42.63°
    angleLerp.put(4.94 + shotOffset, 0.7940919579); // 45.5°

    velocityLerp.put(0.2548162392, 5.628777321); // 14.6°
    velocityLerp.put(0.2715680592, 6.030438178); // 15.56°
    velocityLerp.put(0.2858797963, 6.540398345); // 16.38°
    velocityLerp.put(0.3053273467, 6.859581163); // 17.5°
    velocityLerp.put(0.3249698457, 6.982560199); // 18.62°
    velocityLerp.put(0.3403288802, 7.134449321); // 19.5°
    velocityLerp.put(0.3589263261, 7.350273863); // 20.56°
    velocityLerp.put(0.3938758705, 7.266528096); // 22.56°
    velocityLerp.put(0.4099923863, 7.321765641); // 23.48°
    velocityLerp.put(0.4257852268, 7.378271886); // 24.39°
    velocityLerp.put(0.4485343888, 7.447889085); // 25.7°
    velocityLerp.put(0.4669553025, 7.528828409); // 26.75°
    velocityLerp.put(0.4839191944, 7.569435279); // 27.727°
    velocityLerp.put(0.5028848730, 7.585611046); // 28.81°
    velocityLerp.put(0.5152020140, 7.609483022); // 29.52°
    velocityLerp.put(0.5327448563, 7.685768651); // 30.53°
    velocityLerp.put(0.5501223593, 7.662851550); // 31.52°
    velocityLerp.put(0.5860784353, 7.654255170); // 33.57°
    velocityLerp.put(0.6083099749, 7.728963391); // 34.85°
    velocityLerp.put(0.6592155526, 7.795200170); // 37.76°
    velocityLerp.put(0.6835955730, 7.772150618); // 39.16°
    velocityLerp.put(0.7133274470, 7.918865415); // 40.87°
    velocityLerp.put(0.7436549058, 7.893666440); // 42.63°
    velocityLerp.put(0.7940919579, 8.159037027); // 45.5°
  }

  public double getVelocity(double xf) {
    double numerator = g * Math.pow(xf, 2);
    double denom1 = -yf + xf * Math.tan(pitch);
    double denom2 = 2 * Math.pow(Math.cos(pitch), 2);
    return Math.sqrt(numerator / (denom1 * denom2));
  }

  public double newGetVelocity(double xf) {
    // double voy = Math.sqrt(2 * g * H);
    // double vx = g * xf / (voy + Math.sqrt(Math.pow(voy, 2) - 2 * g * yf));
    // return Math.sqrt(Math.pow(vx, 2) + Math.pow(voy, 2));
    double angle = angleLerp.get(xf);
    return velocityLerp.get(angle);
  }

  public double newGetPitch(double xf) {
    // double voy = Math.sqrt(2 * g * H);
    // double vx = g * xf / (voy + Math.sqrt(Math.pow(voy, 2) - 2 * g * yf));
    // return Math.atan(voy / vx);
    // return HoodConstants.angleLerp.get(xf);
    return angleLerp.get(xf);
  }

  public double getVelocity8(ChassisSpeeds vr, double xf) {
    double a = 0.1;
    double b = 0;
    return getVelocity(xf);
  }

  public double getPitch(double v, double xf) {
    // double A = -g * Math.pow(xf, 2) / (2 * Math.pow(v, 2));
    // double B = xf;
    // double C = A - yf;

    // double theta = Math.atan((-B - Math.sqrt(Math.pow(B, 2) - 4 * A * C)) / (2 * A));

    // return theta;
    // return HoodConstants.angleLerp.get(xf);
    return 5;
  }

  public double getYaw(Pose2d robotPose, double hubX) {
    Transform2d turretOffsetTransform2d =
        new Transform2d(turretOffset.getX(), turretOffset.getY(), new Rotation2d());
    Pose2d turretPositionPose2d = robotPose.plus(turretOffsetTransform2d);

    double deltax = hubX - turretPositionPose2d.getX();
    double deltay = Constants.TargetConstants.hub.getY() - turretPositionPose2d.getY();

    double initTheta = Math.PI - Math.atan2(deltay, -deltax);

    double theta = (initTheta - robotPose.getRotation().getRadians());

    return mod(theta);
  }

  public double getYaw(Pose2d robotPose, double xOffset, double yOffset) {
    Transform2d turretOffsetTransform2d =
        new Transform2d(turretOffset.getX(), turretOffset.getY(), new Rotation2d());
    Pose2d turretPositionPose2d = robotPose.plus(turretOffsetTransform2d);

    double deltax = Constants.TargetConstants.hub.getX() - turretPositionPose2d.getX() + xOffset;
    double deltay = Constants.TargetConstants.hub.getY() - turretPositionPose2d.getY() + yOffset;

    double initTheta = Math.PI - Math.atan2(deltay, -deltax);

    double theta = (initTheta - robotPose.getRotation().getRadians());

    return mod(theta);
  }

  public double getPassingYaw(Pose2d robotPose, double xOffset, double yOffset) {
    Transform2d turretOffsetTransform2d =
        new Transform2d(turretOffset.getX(), turretOffset.getY(), new Rotation2d());
    Pose2d turretPositionPose2d = robotPose.plus(turretOffsetTransform2d);

    double deltax = Constants.TargetConstants.pass1.getX() - turretPositionPose2d.getX() + xOffset;
    double deltay = Constants.TargetConstants.pass1.getY() - turretPositionPose2d.getY() + yOffset;

    double initTheta = Math.PI - Math.atan2(deltay, -deltax);

    double theta = (initTheta - robotPose.getRotation().getRadians());

    return mod(theta);
  }

  public double getRobotRelativeYaw(Pose2d robotPose, double hubX) {
    return getYaw(robotPose, hubX) + 0 * robotPose.getRotation().getRadians();
  }

  public double getTime(double xf) {
    return xf / (getVelocity(xf) * Math.cos(pitch));
  }

  public double getMovingVelocity(double xf, ChassisSpeeds Vr, Pose2d robotPose) {
    return 1;
  }

  public double getMovingYaw(double xf, ChassisSpeeds Vr, Pose2d robotPose) {
    return 1;
  }

  public double mod(double angle) {
    return ((angle % (2 * Math.PI)) + (2 * Math.PI)) % (2 * Math.PI);
  }
}
