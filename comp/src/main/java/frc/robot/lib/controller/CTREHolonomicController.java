package frc.robot.lib.controller;

import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.lib.RJLog;

/**
 * Custom version of a @HolonomicDriveController specifically for following PathPlanner paths
 *
 * <p>This controller adds the following functionality over the WPILib version: - calculate() method
 * takes in a PathPlannerState directly - Continuous input is automatically enabled for the rotation
 * controller - Holonomic angular velocity is used as a feedforward for the rotation controller,
 * which no longer needs to be a @ProfiledPIDController
 */
public class CTREHolonomicController {
  private final PhoenixPIDController xController;
  private final PhoenixPIDController yController;
  private final PhoenixPIDController rotationController;

  private Pose2d poseError = new Pose2d();
  private Pose2d tolerance = new Pose2d();
  private boolean isEnabled = true;

  /**
   * Constructs a PPHolonomicDriveController
   *
   * @param xController A PID controller to respond to error in the field-relative X direction
   * @param yController A PID controller to respond to error in the field-relative Y direction
   * @param rotationController A PID controller to respond to error in rotation
   */
  public CTREHolonomicController(
      PhoenixPIDController xController,
      PhoenixPIDController yController,
      PhoenixPIDController rotationController) {
    this.xController = xController;
    this.yController = yController;
    this.rotationController = rotationController;

    // Auto-configure continuous input for rotation controller
    this.rotationController.enableContinuousInput(-Math.PI, Math.PI);
  }

  /**
   * Returns true if the pose error is within tolerance of the reference.
   *
   * @return True if the pose error is within tolerance of the reference.
   */
  public boolean atReference() {
    Translation2d translationTolerance = this.tolerance.getTranslation();
    Rotation2d rotationTolerance = this.tolerance.getRotation();

    return Math.abs(this.poseError.getX()) < translationTolerance.getX()
        && Math.abs(this.poseError.getY()) < translationTolerance.getY()
        && Math.abs(this.poseError.getRotation().getRadians()) < rotationTolerance.getRadians();
  }

  /**
   * Sets the pose error whic is considered tolerance for use with atReference()
   *
   * @param tolerance The pose error which is tolerable
   */
  public void setTolerance(Pose2d tolerance) {
    this.tolerance = tolerance;
  }

  /**
   * Enables and disables the controller for troubleshooting. When calculate() is called on a
   * disabled controller, only feedforward values are returned.
   *
   * @param enabled If the controller is enabled or not
   */
  public void setEnabled(boolean enabled) {
    this.isEnabled = enabled;
  }

  /**
   * Calculates the next output of the holonomic drive controller
   *
   * @param currentPose The current pose
   * @param referenceState The desired trajectory state
   * @return The next output of the holonomic drive controller
   */
  public ChassisSpeeds calculate(
      Pose2d currentPose,
      Pose2d referencePose,
      ChassisSpeeds referenceSpeeds,
      double currentTimestamp) {
    double xFF = referenceSpeeds.vx;
    double yFF = referenceSpeeds.vy;
    double rotationFF = referenceSpeeds.omega;

    this.poseError = referencePose.relativeTo(currentPose);

    if (!this.isEnabled) {
      return new ChassisSpeeds(xFF, yFF, rotationFF).toRobotRelative(currentPose.getRotation());
    }

    double xFeedback =
        this.xController.calculate(currentPose.getX(), referencePose.getX(), currentTimestamp);
    double yFeedback =
        this.yController.calculate(currentPose.getY(), referencePose.getY(), currentTimestamp);
    double rotationFeedback =
        this.rotationController.calculate(
            currentPose.getRotation().getRadians(),
            referencePose.getRotation().getRadians(),
            currentTimestamp);

    RJLog.log("CTREController/X", xFeedback);
    RJLog.log("CTREController/Y", yFeedback);
    RJLog.log("CTREController/Theta", rotationFeedback);

    return new ChassisSpeeds( xFF + xFeedback, yFF + yFeedback, rotationFF + rotationFeedback).toRobotRelative(currentPose.getRotation());
  }
}
