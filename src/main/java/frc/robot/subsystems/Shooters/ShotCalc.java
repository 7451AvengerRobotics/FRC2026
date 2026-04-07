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
  public double pitch = Math.toRadians(60);
  Transform3d turretOffset = new Transform3d();
  InterpolatingDoubleTreeMap angleLerp = new InterpolatingDoubleTreeMap();
  InterpolatingDoubleTreeMap velocityLerp = new InterpolatingDoubleTreeMap();

  public ShotCalc(Transform3d turretOffset) {
    this.turretOffset = turretOffset;
    angleLerp.put(1.0687, 0.0000000000); // 0°
    angleLerp.put(1.42, 0.0167551608); // 0.96°
    angleLerp.put(1.87, 0.0310668979); // 1.78°
    angleLerp.put(2.24, 0.0506145483); // 2.9°
    angleLerp.put(2.465, 0.0701620988); // 4.02°
    angleLerp.put(2.7, 0.0855211333); // 4.9°
    angleLerp.put(3.03, 0.1040235792); // 5.96°
    angleLerp.put(3.15, 0.1389731236); // 7.96°
    angleLerp.put(3.3, 0.1549896394); // 8.88°
    angleLerp.put(3.45, 0.1708874284); // 9.79°
    angleLerp.put(3.65, 0.1937315390); // 11.1°
    angleLerp.put(3.84, 0.2120575042); // 12.15°
    angleLerp.put(3.97, 0.2291163447); // 13.127°
    angleLerp.put(4.07, 0.2480819233); // 14.21°
    angleLerp.put(4.15, 0.2603990643); // 14.92°
    angleLerp.put(4.32, 0.2780368552); // 15.93°
    angleLerp.put(4.34, 0.2953193067); // 16.92°
    angleLerp.put(4.41, 0.3311804342); // 18.97°
    angleLerp.put(4.56, 0.3534119738); // 20.25°
    angleLerp.put(4.69, 0.4042226029); // 23.16°
    angleLerp.put(4.63, 0.4287925719); // 24.56°
    angleLerp.put(4.83, 0.4587193445); // 26.27°
    angleLerp.put(4.69, 0.4891418519); // 28.03°
    angleLerp.put(4.94, 0.5393965113); // 30.9°

    velocityLerp.put(0.0000000000, 5.628777321); // 0°
    velocityLerp.put(0.0167551608, 6.030438178); // 0.96°
    velocityLerp.put(0.0310668979, 6.540398345); // 1.78°
    velocityLerp.put(0.0506145483, 6.859581163); // 2.9°
    velocityLerp.put(0.0701620988, 6.982560199); // 4.02°
    velocityLerp.put(0.0855211333, 7.134449321); // 4.9°
    velocityLerp.put(0.1040235792, 7.350273863); // 5.96°
    velocityLerp.put(0.1389731236, 7.266528096); // 7.96°
    velocityLerp.put(0.1549896394, 7.321765641); // 8.88°
    velocityLerp.put(0.1708874284, 7.378271886); // 9.79°
    velocityLerp.put(0.1937315390, 7.447889085); // 11.1°
    velocityLerp.put(0.2120575042, 7.528828409); // 12.15°
    velocityLerp.put(0.2291163447, 7.569435279); // 13.127°
    velocityLerp.put(0.2480819233, 7.585611046); // 14.21°
    velocityLerp.put(0.2603990643, 7.609483022); // 14.92°
    velocityLerp.put(0.2780368552, 7.685768651); // 15.93°
    velocityLerp.put(0.2953193067, 7.662851550); // 16.92°
    velocityLerp.put(0.3311804342, 7.654255170); // 18.97°
    velocityLerp.put(0.3534119738, 7.728963391); // 20.25°
    velocityLerp.put(0.4042226029, 7.795200170); // 23.16°
    velocityLerp.put(0.4287925719, 7.772150618); // 24.56°
    velocityLerp.put(0.4587193445, 7.918865415); // 26.27°
    velocityLerp.put(0.4891418519, 7.893666440); // 28.03°
    velocityLerp.put(0.5393965113, 8.159037027); // 30.9°
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
