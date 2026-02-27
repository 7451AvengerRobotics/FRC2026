package frc.robot.subsystems.SimFiles;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Constants.TargetConstants;

public class FuelSim {
  double vDesiredX;
  double vDesiredY;
  double vVertical;
  Pose2d initialPos;

  double time = 0;
  double accel = 9.8;
  double height = 0.5;
  Translation2d hub = TargetConstants.hub;

  boolean killValue = false;
  double killTimer;

  /** Ball travels at v_desired in field frame. vVertical is initial upward speed for arc. */
  public FuelSim(
      double vDesiredX, double vDesiredY, double vVertical, Pose2d initialPos) {
    this.vDesiredX = vDesiredX;
    this.vDesiredY = vDesiredY;
    this.vVertical = vVertical;
    this.initialPos = initialPos;
  }

  public Pose3d getPose() {
    return new Pose3d(new Translation3d(getX(), getY(), Math.max(getZ(), 0)), new Rotation3d());
  }

  public double getX() {
    return initialPos.getX() + vDesiredX * time;
  }

  public double getY() {
    return initialPos.getY() + vDesiredY * time;
  }

  public double getZ() {
    return height + vVertical * time - 0.5 * accel * time * time;
  }

  public void update(double dt) {
    time += dt;
  }

  // Removes the ball after __ seconds. Ball scores when within hub opening (72in = 1.829m).
  public boolean isDead() {
    double dx = Math.abs(hub.getX() - getX());
    double dy = Math.abs(hub.getY() - getY());
    // Hub opening ~72in diameter; use 0.6m tolerance for scoring
    boolean inHubXY = dx < 0.6 && dy < 0.6;
    boolean inHubHeight = getZ() > 1.5 && getZ() < 2.0;
    boolean startTimerCondition = (inHubXY && inHubHeight) || time > 5;
    if (startTimerCondition) {
      if (!killValue) {
        killValue = true;
        killTimer = time;
      }
    }
    return killTimer != 0 ? (time - killTimer) > 0.5 : false;
  }
}
