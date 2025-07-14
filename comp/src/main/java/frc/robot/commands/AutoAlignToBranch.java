package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.ApplyFieldSpeeds;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.FieldConstants.Side;
import frc.robot.lib.RJLog;
import frc.robot.subsystems.Drive;

public class AutoAlignToBranch extends Command {
  private final Drive drive;
  private final Side side;

  private final String prefix;
  private final FieldConstants fieldConstants = FieldConstants.getInstance();

  public final ProfiledPIDController xController;
  public final ProfiledPIDController yController;
  public final ProfiledPIDController rotationController;

  private Pose2d targetPose;

  private final ApplyFieldSpeeds request =
      new ApplyFieldSpeeds()
          .withDriveRequestType(DriveRequestType.Velocity)
          .withSteerRequestType(SteerRequestType.Position);

  public AutoAlignToBranch(
      Drive drive,
      Side side,
      TrapezoidProfile.Constraints xConstraints,
      TrapezoidProfile.Constraints yConstraints,
      TrapezoidProfile.Constraints rotationConstraints,
      String prefix) {
    this.drive = drive;
    this.side = side;
    this.prefix = prefix;

    this.xController = new ProfiledPIDController(0, 0, 0, xConstraints);
    this.yController = new ProfiledPIDController(0, 0, 0, yConstraints);
    this.rotationController = new ProfiledPIDController(0, 0, 0, rotationConstraints);
    this.rotationController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(drive);
  }

  @Override
  public void initialize() {
    var currentState = drive.getStateCopy();

    var closestTag =
        fieldConstants.getClosestReefTag(currentState.Pose, DriverStation.getAlliance().get());
    targetPose = fieldConstants.RobotPose(closestTag, side);

    ChassisSpeeds currentSpeeds =
        currentState.Speeds.toFieldRelative(currentState.Pose.getRotation());

    xController.reset(currentState.Pose.getX(), currentSpeeds.vx);
    yController.reset(currentState.Pose.getY(), currentSpeeds.vy);
    rotationController.reset(currentState.Pose.getRotation().getRadians(), currentSpeeds.omega);

    RJLog.log(String.format("%s/InitialState", prefix), currentState);
    RJLog.log(String.format("%s/TargetTag", prefix), closestTag);
    RJLog.log(String.format("%s/GoalPose", prefix), targetPose);
  }

  @Override
  public void execute() {
    Pose2d currentPose = drive.getStateCopy().Pose;

    double xFeedback = xController.calculate(currentPose.getX(), targetPose.getX());
    double yFeedback = yController.calculate(currentPose.getY(), targetPose.getY());
    double rotFeedback =
        rotationController.calculate(
            currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());

    double xFF = xController.getSetpoint().velocity;
    double yFF = yController.getSetpoint().velocity;
    double rotFF = rotationController.getSetpoint().velocity;

    double xVel = xFF + xFeedback;
    double yVel = yFF + yFeedback;
    double rotVel = rotFF + rotFeedback;

    var currentTargetPose =
        new Pose2d(
            xController.getSetpoint().position,
            yController.getSetpoint().position,
            Rotation2d.fromRadians(rotationController.getSetpoint().position));
    var currentTargetSpeeds = new ChassisSpeeds(xVel, yVel, rotVel);

    RJLog.log(String.format("%s/ImmediateTargetPose", prefix), currentTargetPose);
    RJLog.log(String.format("%s/ImmediateTargetSpeeds", prefix), currentTargetSpeeds);

    drive.setControl(request.withSpeeds(currentTargetSpeeds));
  }

  @Override
  public boolean isFinished() {
    return xController.atGoal() && yController.atGoal() && rotationController.atGoal();
  }
}
