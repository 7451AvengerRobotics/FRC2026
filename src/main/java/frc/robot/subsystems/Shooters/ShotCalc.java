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
  double shotOffset = 0;
  public double pitch = Math.toRadians(60);
  Transform3d turretOffset = new Transform3d();
  InterpolatingDoubleTreeMap angleLerp = new InterpolatingDoubleTreeMap();
  InterpolatingDoubleTreeMap velocityLerp = new InterpolatingDoubleTreeMap();

  public ShotCalc(Transform3d turretOffset) {
    this.turretOffset = turretOffset;
    // angleLerp: distance (m) -> angle (rad)
    angleLerp.put(2.38 + shotOffset, 1.3028882870); // 74.65°
    angleLerp.put(2.52 + shotOffset, 1.2777555450); // 73.21°
    angleLerp.put(2.63 + shotOffset, 1.2575097260); // 72.05°
    angleLerp.put(2.72 + shotOffset, 1.2384856370); // 70.96°
    angleLerp.put(2.92 + shotOffset, 1.2264428650); // 70.27°
    angleLerp.put(3.07 + shotOffset, 1.1974704000); // 68.61°
    angleLerp.put(3.15 + shotOffset, 1.1705923290); // 67.07°
    angleLerp.put(3.39 + shotOffset, 1.1506955760); // 65.93°
    angleLerp.put(3.64 + shotOffset, 1.1218976430); // 64.28°
    angleLerp.put(3.82 + shotOffset, 1.0993828960); // 62.99°
    angleLerp.put(4.25 + shotOffset, 1.0796606750); // 61.86°
    angleLerp.put(4.41 + shotOffset, 1.0564477960); // 60.53°
    angleLerp.put(4.72 + shotOffset, 1.0088003080); // 57.80°
    angleLerp.put(4.99 + shotOffset, 0.9848892969); // 56.43°
    angleLerp.put(5.17 + shotOffset, 0.9660397410); // 55.35°
    angleLerp.put(5.29 + shotOffset, 0.9180431865); // 52.60°
    angleLerp.put(5.41 + shotOffset, 0.8677777041); // 49.72°
    angleLerp.put(5.58 + shotOffset, 0.8513716091); // 48.78°
    angleLerp.put(5.66 + shotOffset, 0.8307767239); // 47.60°
    angleLerp.put(5.79 + shotOffset, 0.8011061267); // 45.90°

    // velocityLerp: angle (rad) -> velocity (m/s)
    velocityLerp.put(1.3072516100, 6.3764160460); // 15.1°
    velocityLerp.put(1.3037609510, 6.9474119980); // 15.3°
    velocityLerp.put(1.2955579040, 7.0199995490); // 15.77°
    velocityLerp.put(1.2941616400, 7.0359599800); // 15.85°
    velocityLerp.put(1.2835151320, 7.1072686120); // 16.46°
    velocityLerp.put(1.2718214260, 7.0846616950); // 17.13°
    velocityLerp.put(1.2603022530, 7.2141174950); // 17.79°
    velocityLerp.put(1.2541936000, 7.2347236220); // 18.14°
    velocityLerp.put(1.2250466020, 7.2439663810); // 19.81°
    velocityLerp.put(1.2107349020, 7.1756909430); // 20.63°
    velocityLerp.put(1.1475539830, 7.1229431880); // 24.25°
    velocityLerp.put(1.1135200630, 7.2300475180); // 26.2°
    velocityLerp.put(1.0960667700, 7.3390407720); // 27.2°
    velocityLerp.put(1.0838494650, 7.2021911380); // 27.9°
    velocityLerp.put(1.0314895880, 7.5326382080); // 30.9°
    velocityLerp.put(1.0005972600, 7.5620423180); // 32.67°
    velocityLerp.put(0.9684832019, 7.8102271870); // 34.51°
    velocityLerp.put(0.9171705219, 7.9621526530); // 37.45°
    velocityLerp.put(0.8749335540, 8.0311150000); // 39.87°
    velocityLerp.put(0.8477064177, 8.1340625400); // 41.43°
    velocityLerp.put(0.8382816397, 8.2305036130); // 41.97°
    velocityLerp.put(0.8014551925, 8.3914965200); // 44.08°
    velocityLerp.put(0.7686430026, 8.5306090010); // 45.96°
    velocityLerp.put(0.7518878418, 8.6155074580); // 46.92°
    velocityLerp.put(0.5846852994, 9.6663236710); // 56.5°
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
    return Math.PI / 2 - angleLerp.get(xf);
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
