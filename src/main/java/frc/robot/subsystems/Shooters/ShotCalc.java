package frc.robot.subsystems.Shooters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TargetConstants;
import frc.robot.subsystems.drive.Drive;

/**
 * Ballistics and aim calculations for shooting at a target (e.g. hub).
 *
 * <p>Uses robot pose and target pose (from {@link #setTarget}) to compute:
 *
 * <ul>
 *   <li>Horizontal range {@code xf} (m) from turret to target
 *   <li>Required launch angle (pitch) for the hood via {@link #getPitchForDistance}
 *   <li>Required launch speed for the shooter via {@link #getVelocity}
 *   <li>Turret yaw (robot-relative angle to aim at target) via {@link #getYaw}
 * </ul>
 *
 * <p>All calculations use the field pose of the hub (or whatever target is set). Call {@link
 * #updateState()} once per cycle before using range/yaw/pitch so xf and field-relative state are
 * current.
 */
public class ShotCalc {
  /** Gravitational acceleration (m/s²). */
  private static final double g = 9.81;
  /** For ballistic quadratic: a = -g. */
  private static final double a = -g;

  private double xf;
  private double yf;
  private ChassisSpeeds vr;
  private double vxr;
  private double vyr;
  /** Fixed pitch (rad) used by {@link #getVelocity(double)} when not using distance-based pitch. */
  private double pitch = Math.toRadians(60);

  private final Transform2d turretOffsetTransform2d;
  private Pose2d turretPositionPose2d;
  private final Drive drive;
  /** Target position in field coordinates (e.g. hub). Used for xf and getYaw. */
  private Translation2d target = TargetConstants.hub;

  public ShotCalc(Drive drive, Transform3d turretOffset) {
    this.drive = drive;
    this.yf = TargetConstants.yf;
    this.turretOffsetTransform2d =
        new Transform2d(turretOffset.getX(), turretOffset.getY(), new Rotation2d());

    turretPositionPose2d = drive.getPose().plus(turretOffsetTransform2d);
  }

  /** Sets the target position in field coordinates (e.g. hub). Affects xf and getYaw. */
  public void setTarget(Translation2d newTarget) {
    target = newTarget;
  }

  /** Horizontal distance (m) from turret to target. Valid after {@link #updateState()}. */
  public double getXf() {
    return this.xf;
  }

  /** Fixed pitch in radians (legacy). Prefer {@link #getPitchForDistance} for hood. */
  public double getPitch() {
    return this.pitch;
  }

  /**
   * Required launch angle (pitch) in radians for the given horizontal range so the ball reaches the
   * target height {@code yf} when launched at the fixed speed {@link
   * ShooterConstants#kFixedLaunchVelocityMetersPerSecond}. Only the hood angle varies with
   * distance; shooter speed is fixed.
   */
  public double getPitchForDistance(double xf) {
    double v = ShooterConstants.kFixedLaunchVelocityMetersPerSecond;
    double A = a * Math.pow(xf, 2) / (2 * Math.pow(v, 2));
    double B = xf;
    double C = A - yf;
    double discriminant = Math.pow(B, 2) - 4 * A * C;
    if (discriminant < 0) {
      return Math.toRadians(60); // fallback when no solution for this speed (e.g. out of range)
    }
    return Math.atan((-B - Math.sqrt(discriminant)) / (2 * A));
  }

  /**
   * Launch speed (m/s) for the given horizontal range. Empirical formula; kept for legacy use (e.g.
   * getVelocity, moving-shot). Hood uses fixed speed via getPitchForDistance.
   */
  private double getVelocityForDistance(double xf) {
    return Math.sqrt(g * yf + g * Math.sqrt(Math.pow(xf, 2) + Math.pow(yf, 2))) + 0.5;
  }

  /** Updates turret position, field-relative chassis speeds, and horizontal range to target. */
  public void updateState() {
    setTarget(drive.getHubPositionForCurrentAlliance());
    turretPositionPose2d = drive.getPose().plus(turretOffsetTransform2d);
    vr =
        ChassisSpeeds.fromRobotRelativeSpeeds(
            drive.getRobotRelativeSpeeds(), drive.getPose().getRotation());
    vxr = vr.vxMetersPerSecond;
    vyr = vr.vyMetersPerSecond;

    xf =
        Math.sqrt(
            Math.pow((target.getX() - turretPositionPose2d.getX()), 2)
                + Math.pow((target.getY() - turretPositionPose2d.getY()), 2));
  }

  /**
   * Launch speed (m/s) for the given horizontal range using the fixed pitch. From projectile
   * motion: v² = g·xf² / (2·cos²(θ)·(xf·tan(θ) − yf)).
   */
  public double getVelocity(double xf) {
    double numerator = g * Math.pow(xf, 2);
    double denom1 = (xf * Math.tan(pitch)) - yf;
    double denom2 = 2 * Math.pow(Math.cos(pitch), 2);
    return Math.sqrt(numerator / (denom1 * denom2));
  }

  /**
   * Turret yaw (rad) in robot frame to aim at the current target. Returned in [0, 2π). Uses the
   * target set by {@link #setTarget} (default hub pose). The turret should rotate to this angle so
   * the shooter points at the hub.
   */
  public double getYaw(Pose2d robotPose) {
    Pose2d turretPose2d = robotPose.plus(turretOffsetTransform2d);

    double deltax = target.getX() - turretPose2d.getX();
    double deltay = target.getY() - turretPose2d.getY();

    double initTheta = Math.PI - Math.atan2(deltay, -deltax);
    double theta = (initTheta - robotPose.getRotation().getRadians());

    return mod(theta);
  }

  /**
   * Returns the equivalent yaw in [0, 2π) that is closest to the current turret angle (shortest
   * path). Use this so the turret rotates back toward zero when past 360° instead of taking the
   * long way.
   *
   * @param robotPose current robot pose
   * @param currentTurretRad current turret angle in radians (same convention as getYaw)
   * @return setpoint in [0, 2π) that points at the target and minimizes |setpoint −
   *     currentTurretRad|
   */
  public double getYawShortestPath(Pose2d robotPose, double currentTurretRad) {
    return getYawEquivalentClosestTo(getYaw(robotPose), currentTurretRad);
  }

  /**
   * Returns the angle in [0, 2π) that is equivalent to targetAngleRad (mod 2π) and closest to
   * currentTurretRad. Use for shortest-path turret control when the turret uses a different
   * convention (e.g. right side uses -yaw).
   */
  public double getYawEquivalentClosestTo(double targetAngleRad, double currentTurretRad) {
    double targetYaw = mod(targetAngleRad);
    double diff = targetYaw - currentTurretRad;
    if (diff > Math.PI) {
      targetYaw -= 2 * Math.PI;
    } else if (diff < -Math.PI) {
      targetYaw += 2 * Math.PI;
    }
    return mod(targetYaw);
  }

  /** Time of flight (s) for the current fixed-pitch shot. */
  public double getTime() {
    return xf / (getVelocity(xf) * Math.cos(pitch));
  }

  /** Launch speed (m/s) adjusted for robot motion (moving shot). */
  public double getMovingVelocity(double xf, ChassisSpeeds Vr, Pose2d robotPose) {
    double vxri = Vr.vxMetersPerSecond;
    double vyri = Vr.vyMetersPerSecond;

    double robotVelocityMag = Math.sqrt(Math.pow(vxri, 2) + Math.pow(vyri, 2));

    double robotVelocityAngle = Math.atan2(vyri, vxri);

    double transformedVelocityAngle = robotVelocityAngle - getYaw(robotPose) + Math.PI / 2;

    double vrxf = Math.cos(transformedVelocityAngle) * robotVelocityMag;

    double vrxHorizontalDisplacement = -vrxf * getTime();

    double newxf = Math.sqrt(Math.pow(xf, 2) + Math.pow(vrxHorizontalDisplacement, 2));

    return getVelocity(newxf);
  }

  /** Turret yaw (rad) adjusted for robot motion so the shot still hits the target. */
  public double getMovingYaw(double xf, ChassisSpeeds Vr, Pose2d robotPose) {
    double vxri = Vr.vxMetersPerSecond;
    double vyri = Vr.vyMetersPerSecond;

    double robotVelocityMag = Math.sqrt(Math.pow(vxri, 2) + Math.pow(vyri, 2));

    double robotVelocityAngle = Math.atan2(vyri, vxri);

    double transformedVelocityAngle = robotVelocityAngle - getYaw(robotPose) + Math.PI / 2;

    double vrxf = Math.cos(transformedVelocityAngle) * robotVelocityMag;

    double vrxHorizontalDisplacement = -vrxf * getTime();

    double yawAdj = Math.atan2(vrxHorizontalDisplacement, xf);

    return mod(getYaw(robotPose) + yawAdj);
  }

  /** Wraps angle to [0, 2π). */
  public double mod(double angle) {
    return ((angle % (2 * Math.PI)) + (2 * Math.PI)) % (2 * Math.PI);
  }
}
