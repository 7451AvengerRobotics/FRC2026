package frc.robot.subsystems.SimFiles;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TargetConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.Shooters.ShotCalc;
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

  ChassisSpeeds VrRobot;
  ChassisSpeeds Vr;
  double vxr;
  double vyr;

  private final List<FuelSim> activeFuel = new ArrayList<>();
  private Translation2d target;
  double yf = 1.329;
  double xf;
  double g = 9.8;
  double a = -g;

  private final ShotCalc shotCalc;

  public TurretSim(Drive drive, Transform3d turretOffset, String name) {
    this.drive = drive;
    this.turretOffset = turretOffset;
    this.name = name;

    turretOffsetTransform2d =
        new Transform2d(turretOffset.getX(), turretOffset.getY(), new Rotation2d());
    turretPositionPose2d = drive.getPose().plus(turretOffsetTransform2d);

    this.shotCalc = new ShotCalc(turretOffset);

    target =
        new Translation2d(
            // Commented is red code
            // 16.54 -
            drive.applyX(TargetConstants.hub.getX()), TargetConstants.hub.getY());
  }

  public void setTarget(Translation2d newTarget) {
    this.target = newTarget;
  }

  public Translation2d getTarget() {
    return this.target;
  }

  public Transform3d getTurretOffset() {
    return this.turretOffset;
  }

  @Override
  public void periodic() {
    turretPositionPose2d = drive.getPose().plus(turretOffsetTransform2d);
    VrRobot = drive.getRobotRelativeSpeeds();
    Vr = ChassisSpeeds.fromRobotRelativeSpeeds(VrRobot, drive.getPose().getRotation());
    vxr = Math.abs(Vr.vxMetersPerSecond) < 0.01 ? 0 : Vr.vxMetersPerSecond;
    vyr = Math.abs(Vr.vyMetersPerSecond) < 0.01 ? 0 : Vr.vyMetersPerSecond;

    xf = this.getXf(0, 0);

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
    Logger.recordOutput("Xf", xf);

    double v0 = shotCalc.newGetVelocity(xf);
    double pitch0 = shotCalc.newGetPitch(xf);
    double yaw0 = shotCalc.getYaw(drive.getPose(), TargetConstants.hub.getX());

    double time = calcShotTime(xf, v0, pitch0);
    double adjustedXf = getXf(-vxr * time, -vyr * time);

    double vf = shotCalc.newGetVelocity(adjustedXf);
    double pitchf = shotCalc.newGetPitch(adjustedXf);
    double yawf = shotCalc.getYaw(drive.getPose(), -vxr * time, -vyr * time);

    Logger.recordOutput("vf_" + name, vf);
    Logger.recordOutput("pitchf_" + name, pitchf * 180 / Math.PI);
    Logger.recordOutput("yawf_" + name, yawf * 180 / Math.PI);
  }

  public double calcYaw() {
    double deltax = target.getX() - turretPositionPose2d.getX() + vxr * TurretConstants.latency;
    double deltay = target.getY() - turretPositionPose2d.getY() + vyr * TurretConstants.latency;
    double initTheta;

    if (deltax == 0) {
      initTheta = 0;
    } else {
      initTheta = Math.PI - Math.atan2(deltay, -deltax);
    }

    double theta = (initTheta - drive.getPose().getRotation().getRadians());

    return mod(theta);
  }

  public double calcYaw(double deltax, double deltay) {
    double initTheta;

    if (deltax == 0) {
      initTheta = 0;
    } else {
      initTheta = Math.PI - Math.atan2(deltay, -deltax);
    }

    double theta = (initTheta - drive.getPose().getRotation().getRadians());

    return mod(theta);
  }

  public double calcYawForSimBall(double yaw) {
    return yaw + drive.getPose().getRotation().getRadians();
  }

  public double velocityYaw() {
    return Math.atan2(vyr, vxr) - Math.PI / 2;
  }

  public double calcVelocity(double xf) {
    double vel = Math.sqrt(g * yf + g * Math.sqrt(Math.pow(xf, 2) + Math.pow(yf, 2))) + 0.5;

    return vel;
  }

  public double calcShotTime(double xf, double v, double pitch) {
    return xf / (v * Math.cos(pitch));
  }

  public void shootBall() {
    double v0 = shotCalc.newGetVelocity(xf);
    double pitch0 = shotCalc.newGetPitch(xf);
    double yaw0 = shotCalc.getYaw(drive.getPose(), drive.applyX(TargetConstants.hub.getX()));

    double time = calcShotTime(xf, v0, pitch0);
    double adjustedXf = getXf(-vxr * time, -vyr * time);

    double vf = shotCalc.newGetVelocity(adjustedXf);
    double pitchf = shotCalc.newGetPitch(adjustedXf);
    double yawf = shotCalc.getYaw(drive.getPose(), -vxr * time, -vyr * time);

    // activeFuel.add(
    //     new FuelSim(
    //         vf,
    //         pitchf,
    //         yawf + drive.getPose().getRotation().getRadians(),
    //         turretPositionPose2d,
    //         Vr.vxMetersPerSecond,
    //         Vr.vyMetersPerSecond));

    activeFuel.add(
        new FuelSim(
            vf,
            pitchf,
            yawf + drive.getPose().getRotation().getRadians(),
            turretPositionPose2d,
            Vr.vxMetersPerSecond,
            Vr.vyMetersPerSecond));
  }

  public double getRequiredYaw() {
    double v0 = shotCalc.newGetVelocity(xf);
    double pitch0 = shotCalc.newGetPitch(xf);

    double time = calcShotTime(xf, v0, pitch0);

    double yawf = shotCalc.getYaw(drive.getPose(), -vxr * time, -vyr * time);

    return yawf;
  }

  public double getRequiredPitch() {
    double v0 = shotCalc.newGetVelocity(xf);
    double pitch0 = shotCalc.newGetPitch(xf);

    double time = calcShotTime(xf, v0, pitch0);
    double adjustedXf = getXf(-vxr * time, -vyr * time);

    double pitchf = shotCalc.newGetPitch(adjustedXf);

    return pitchf;
  }

  public double getMovingPitch() {
    /**
     * Procedure: 1. Get the required angle via lerp table with xf 2. Get the velocity that comes
     * from that angle through regression 3. Get the time for this shot 4. Repeat the following
     * steps 5 times - Get the adjustedXf with the time - Get the angle for that - Get the velocity
     * for that - Get the time for that shot 5. Using the last time, get the last adjustedXf, and
     * find angle 6. return angle
     */
    double v0 = shotCalc.newGetVelocity(xf);
    double pitch0 = shotCalc.newGetPitch(xf);

    double time = calcShotTime(xf, v0, pitch0);
    double adjustedXf = getXf(-vxr * time, -vyr * time);

    double pitchf = shotCalc.newGetPitch(adjustedXf);

    for(int i = 0; i < 5; i++) {
      pitch0 = pitchf;
      time = calcShotTime(xf, v0, pitch0);
      adjustedXf = getXf(-vxr * time, -vyr * time);
      
      pitchf = shotCalc.newGetPitch(adjustedXf);
    }

    return pitchf;
  }

  public Command shootBallCommand() {
    return runOnce(
        () -> {
          shootBall();
        });
  }

  public double getRequiredVelocity() {
    double v0 = shotCalc.newGetVelocity(xf);
    double pitch0 = shotCalc.newGetPitch(xf);

    double time = calcShotTime(xf, v0, pitch0);
    double adjustedXf = getXf(-vxr * time, -vyr * time);

    double vf = shotCalc.newGetVelocity(adjustedXf);

    return vf;
  }

  public double getColumbusVelocity() {
    return shotCalc.getVelocity(xf);
  }

  public double getXf(double xOffset, double yOffset) {
    xf =
        Math.sqrt(
            Math.pow((drive.applyX(target.getX()) - turretPositionPose2d.getX() + xOffset), 2)
                + Math.pow((target.getY() - turretPositionPose2d.getY() + yOffset), 2));

    return xf;
  }

  // Helper Function:
  public double mod(double angle) {
    return ((angle % (2 * Math.PI)) + (2 * Math.PI)) % (2 * Math.PI);
  }
}
