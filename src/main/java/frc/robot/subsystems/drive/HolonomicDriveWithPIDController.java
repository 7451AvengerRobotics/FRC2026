package frc.robot.subsystems.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class HolonomicDriveWithPIDController {

  private final PIDController xController;
  private final PIDController yController;
  private final PIDController rotationController;
  private final double scale;
  private final double rotationScale;

  public HolonomicDriveWithPIDController(
      final PIDController xController,
      final PIDController yController,
      final PIDController rotationController,
      final double scale,
      final Pose2d poseTolerance,
      final double rotationScale) {
    this.xController = xController;
    this.xController.setTolerance(poseTolerance.getX(), poseTolerance.getX() * 1.5);

    this.yController = yController;
    this.yController.setTolerance(poseTolerance.getY(), poseTolerance.getX() * 1.5);

    this.rotationController = rotationController;
    this.rotationController.enableContinuousInput(-Math.PI, Math.PI);

    this.scale = scale;
    this.rotationScale = rotationScale;
  }

  public void reset(final Pose2d currentPose, final ChassisSpeeds robotRelativeSpeeds) {
    xController.reset();
    yController.reset();
    rotationController.reset();
  }

  public boolean atReference() {
    return xController.atSetpoint() && yController.atSetpoint() && rotationController.atSetpoint();
  }

  public void setTolerance(final Pose2d poseTolerance) {
    this.xController.setTolerance(poseTolerance.getX(), poseTolerance.getX() * 1.5);
    this.yController.setTolerance(poseTolerance.getY(), poseTolerance.getY() * 1.5);
  }

  public boolean xReferenceReached() {
    return xController.atSetpoint();
  }

  public boolean yReferenceReached() {
    return yController.atSetpoint();
  }

  public boolean rotationReferenceReached() {
    return rotationController.atSetpoint();
  }

  public ChassisSpeeds calculate(final Pose2d currentPose, final Pose2d targetPose) {
    final double xFeedback = xController.calculate(currentPose.getX(), targetPose.getX());
    final double yFeedback = yController.calculate(currentPose.getY(), targetPose.getY());

    final double rotationFeedback =
        rotationController.calculate(
            currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());

    return ChassisSpeeds.fromFieldRelativeSpeeds(
        xFeedback * scale,
        yFeedback * scale,
        rotationFeedback * rotationScale,
        currentPose.getRotation());
  }
}
