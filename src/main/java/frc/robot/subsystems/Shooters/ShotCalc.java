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
    angleLerp.put(1.0687, 1.315978256);
    angleLerp.put(1.42,   1.299223095);
    angleLerp.put(1.87,   1.284911395);
    angleLerp.put(2.24,   1.265363708);
    angleLerp.put(2.465,  1.24581602);
    angleLerp.put(2.7,    1.230457123);
    angleLerp.put(3.03,   1.211956633);
    angleLerp.put(3.15,   1.177050048);
    angleLerp.put(3.3,    1.160993018);
    angleLerp.put(3.45,   1.145110522);
    angleLerp.put(3.65,   1.122246709);
    angleLerp.put(3.84,   1.103920752);
    angleLerp.put(3.97,   1.086868885);
    angleLerp.put(4.07,   1.067966969);
    angleLerp.put(4.15,   1.055575132);
    angleLerp.put(4.32,   1.037947306);
    angleLerp.put(4.34,   1.020668547);
    angleLerp.put(4.41,   0.9848892969);
    angleLerp.put(4.56,   0.9625490825);
    angleLerp.put(4.69,   0.9117600012);
    angleLerp.put(4.63,   0.8873253917);
    angleLerp.put(4.83,   0.8574802615);
    angleLerp.put(4.69,   0.8267624667);
    angleLerp.put(4.94,   0.7766715171);

    velocityLerp.put(1.315978256,  5.628777321);
    velocityLerp.put(1.299223095,  6.030438178);
    velocityLerp.put(1.284911395,  6.540398345);
    velocityLerp.put(1.265363708,  6.859581163);
    velocityLerp.put(1.24581602,   6.982560199);
    velocityLerp.put(1.230457123,  7.134449321);
    velocityLerp.put(1.211956633,  7.350273863);
    velocityLerp.put(1.177050048,  7.266528096);
    velocityLerp.put(1.160993018,  7.321765641);
    velocityLerp.put(1.145110522,  7.378271886);
    velocityLerp.put(1.122246709,  7.447889085);
    velocityLerp.put(1.103920752,  7.528828409);
    velocityLerp.put(1.086868885,  7.569435279);
    velocityLerp.put(1.067966969,  7.585611046);
    velocityLerp.put(1.055575132,  7.609483022);
    velocityLerp.put(1.037947306,  7.685768651);
    velocityLerp.put(1.020668547,  7.662851550);
    velocityLerp.put(0.9848892969, 7.654255170);
    velocityLerp.put(0.9625490825, 7.728963391);
    velocityLerp.put(0.9117600012, 7.795200170);
    velocityLerp.put(0.8873253917, 7.772150618);
    velocityLerp.put(0.8574802615, 7.918865415);
    velocityLerp.put(0.8267624667, 7.893666440);
    velocityLerp.put(0.7766715171, 8.159037027);
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
    return 4000;
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
