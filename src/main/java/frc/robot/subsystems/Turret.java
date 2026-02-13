package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class Turret extends SubsystemBase {
  Drive drive;
  Transform3d turretOffset;
  String name;
  Transform2d turretOffsetTransform2d;
  Pose2d turretPositionPose2d;

  private final List<FuelSim> activeFuel = new ArrayList<>();
  Translation2d hub = new Translation2d(11.915, 4.035);
  double yf = 1.329;
  double xf;
  double g = 9.8;
  double a = -g;

  public Turret(Drive drive, Transform3d turretOffset, String name) {
    this.drive = drive;
    this.turretOffset = turretOffset;
    this.name = name;

    turretOffsetTransform2d =
        new Transform2d(turretOffset.getX(), turretOffset.getY(), new Rotation2d());
    turretPositionPose2d = drive.getPose().plus(turretOffsetTransform2d);
  }

  @Override
  public void periodic() {
    turretPositionPose2d = drive.getPose().plus(turretOffsetTransform2d);

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
  }

  public double calcYaw() {
    double deltax = hub.getX() - turretPositionPose2d.getX();
    double deltay = hub.getY() - turretPositionPose2d.getY();
    double initTheta;

    if (deltax == 0) {
      initTheta = 0;
    } else {
      initTheta = Math.atan2(deltay, deltax);
    }

    double theta = (initTheta - drive.getPose().getRotation().getRadians());

    return theta;
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
            Math.pow((hub.getX() - turretPositionPose2d.getX()), 2)
                + Math.pow((hub.getY() - turretPositionPose2d.getY()), 2));

    double v0 = calcVelocity(xf);
    double pitch0 = calcPitch(v0, xf);
    double yaw0 = calcYaw();

    activeFuel.add(new FuelSim(v0, pitch0, yaw0, turretPositionPose2d));
  }

  public Command shootBallCommand() {
    return runOnce(
        () -> {
          shootBall();
        });
  }

  //Helper Function:
  public double mod(double angle) {
    return ((angle % 360) + 360) % 360;
  }

}
