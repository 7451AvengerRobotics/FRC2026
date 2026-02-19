package frc.robot.subsystems.SimFiles;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class FuelSim {
  double pitch;
  double velocity;
  Pose2d initialPos;
  double yaw;

  double interval = 0.02;
  double time = 0;
  double accel = 9.8;
  double height = 0.5;
  Translation2d hub = new Translation2d(11.915, 4.035);

  double xChange;
  double yChange;

  boolean killValue = false;
  double killTimer;

  public FuelSim(double velocity, double pitch, double yaw, Pose2d initialPos) {
    this.velocity = velocity;
    this.pitch = pitch;
    this.yaw = yaw;
    this.initialPos = initialPos;
  }

  public Pose3d getPose() {
    return new Pose3d(new Translation3d(getX(), getY(), Math.max(getZ(), 0)), new Rotation3d());
  }

  public double getX() {
    return initialPos.getX() + velocity * Math.cos(pitch) * Math.cos(yaw) * time;
  }

  public double getY() {
    return initialPos.getY() + velocity * Math.cos(pitch) * Math.sin(yaw) * time;
  }

  public double getZ() {
    return (-0.5 * accel * time * time) + (velocity * Math.sin(pitch) * time) + height;
  }

  public void update(double dt) {
    time += dt;
  }

  // Removes the ball after __ seconds
  public boolean isDead() {
    boolean startTimerCondition =
        ((hub.getX() - getX()) < 0.1
                && (hub.getX() - getX()) > -0.1
                && (hub.getY() - getY()) < 0.1
                && (hub.getY() - getY()) > -0.1
                && getZ() < 1.829)
            || time > 5;
    if (startTimerCondition) {
      if (!killValue) {
        killValue = true;
        killTimer = time;
      }
    }
    SmartDashboard.putNumber("Yaw Theta", yaw * 180 / Math.PI);
    return killTimer != 0 ? (time - killTimer) > 0.5 : false;
  }
}
