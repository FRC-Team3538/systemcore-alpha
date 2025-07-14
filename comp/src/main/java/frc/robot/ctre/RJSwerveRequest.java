package frc.robot.ctre;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import choreo.trajectory.SwerveSample;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveControlParameters;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.lib.controller.CTREHolonomicController;

public class RJSwerveRequest {
  /**
   * Drives the swerve drivetrain in a field-centric manner, maintaining a specified heading angle
   * to ensure the robot is facing the desired point
   */
  public static class RJFieldCentricFacingPoint implements SwerveRequest {
    /**
     * The velocity in the X direction, in m/s. X is defined as forward according to WPILib
     * convention, so this determines how fast to travel forward.
     */
    public double VelocityX = 0;

    /**
     * The velocity in the Y direction, in m/s. Y is defined as to the left according to WPILib
     * convention, so this determines how fast to travel to the left.
     */
    public double VelocityY = 0;

    /**
     * The desired position to face. (0, 0) is the left corner of the blue alliance station wall
     * when in the driver station, according to WPILib convention.
     */
    public Translation2d TargetPoint = new Translation2d();

    /**
     * An offset to the face of the robot which targets the TargetPoint. Useful when the non-front
     * face of the robot should face TargetPoint.
     */
    public Rotation2d HeadingOffset = new Rotation2d();

    /** The allowable deadband of the request, in m/s. */
    public double Deadband = 0;

    /** The rotational deadband of the request, in radians per second. */
    public double RotationalDeadband = 0;

    /**
     * The center of rotation the robot should rotate around. This is (0,0) by default, which will
     * rotate around the center of the robot.
     */
    public Translation2d CenterOfRotation = new Translation2d();

    /** The type of control request to use for the drive motor. */
    public SwerveModule.DriveRequestType DriveRequestType =
        SwerveModule.DriveRequestType.OpenLoopVoltage;

    /** The type of control request to use for the steer motor. */
    public SwerveModule.SteerRequestType SteerRequestType = SwerveModule.SteerRequestType.Position;

    /**
     * Whether to desaturate wheel speeds before applying. For more information, see the
     * documentation of {@link SwerveDriveKinematics#desaturateWheelSpeeds}.
     */
    public boolean DesaturateWheelSpeeds = true;

    /** The perspective to use when determining which direction is forward. */
    public ForwardPerspectiveValue ForwardPerspective = ForwardPerspectiveValue.OperatorPerspective;

    /**
     * The PID controller used to maintain the desired heading. Users can specify the PID gains to
     * change how aggressively to maintain heading.
     *
     * <p>This PID controller operates on heading radians and outputs a target rotational rate in
     * radians per second. Note that continuous input should be enabled on the range [-pi, pi].
     */
    public PhoenixPIDController HeadingController = new PhoenixPIDController(0, 0, 0);

    private final FieldCentric m_fieldCentric = new FieldCentric();

    public RJFieldCentricFacingPoint() {
      HeadingController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public StatusCode apply(
        SwerveControlParameters parameters, SwerveModule<?, ?, ?>... modulesToApply) {
      Translation2d pointToFace = TargetPoint;
      Translation2d pointOffset = pointToFace.minus(parameters.currentPose.getTranslation());
      Rotation2d headingToFace =
          pointOffset.getAngle().plus(Rotation2d.fromRotations(0.5)).plus(HeadingOffset);

      var angularVelocity = 0.0;
      var top = VelocityY * pointOffset.getX() - VelocityX * pointOffset.getY();
      var bottom =
          pointOffset.getX() * pointOffset.getX() + pointOffset.getY() * pointOffset.getY();

      if (bottom > 1e-9) {
        angularVelocity = top / bottom;
      }

      double toApplyOmega =
          -angularVelocity
              + HeadingController.calculate(
                  parameters.currentPose.getRotation().getRadians(),
                  headingToFace.getRadians(),
                  parameters.timestamp);

      ChassisSpeeds toApply =
          ChassisSpeeds.fromFieldRelativeSpeeds(VelocityX, VelocityY, toApplyOmega, HeadingOffset);

      // Todo - maybe I need to do something with the forward perspective?
      return m_fieldCentric
          .withVelocityX(toApply.vxMetersPerSecond)
          .withVelocityY(toApply.vyMetersPerSecond)
          .withRotationalRate(toApply.omegaRadiansPerSecond)
          .withDeadband(Deadband)
          .withRotationalDeadband(RotationalDeadband)
          .withCenterOfRotation(CenterOfRotation)
          .withDriveRequestType(DriveRequestType)
          .withSteerRequestType(SteerRequestType)
          .withDesaturateWheelSpeeds(DesaturateWheelSpeeds)
          .withForwardPerspective(ForwardPerspective)
          .apply(parameters, modulesToApply);
    }

    /**
     * Modifies the VelocityX parameter and returns itself.
     *
     * <p>The velocity in the X direction, in m/s. X is defined as forward according to WPILib
     * convention, so this determines how fast to travel forward.
     *
     * @param newVelocityX Parameter to modify
     * @return this object
     */
    public RJFieldCentricFacingPoint withVelocityX(double newVelocityX) {
      this.VelocityX = newVelocityX;
      return this;
    }

    /**
     * Modifies the VelocityX parameter and returns itself.
     *
     * <p>The velocity in the X direction, in m/s. X is defined as forward according to WPILib
     * convention, so this determines how fast to travel forward.
     *
     * @param newVelocityX Parameter to modify
     * @return this object
     */
    public RJFieldCentricFacingPoint withVelocityX(LinearVelocity newVelocityX) {
      this.VelocityX = newVelocityX.in(MetersPerSecond);
      return this;
    }

    /**
     * Modifies the VelocityY parameter and returns itself.
     *
     * <p>The velocity in the Y direction, in m/s. Y is defined as to the left according to WPILib
     * convention, so this determines how fast to travel to the left.
     *
     * @param newVelocityY Parameter to modify
     * @return this object
     */
    public RJFieldCentricFacingPoint withVelocityY(double newVelocityY) {
      this.VelocityY = newVelocityY;
      return this;
    }

    /**
     * Modifies the VelocityY parameter and returns itself.
     *
     * <p>The velocity in the Y direction, in m/s. Y is defined as to the left according to WPILib
     * convention, so this determines how fast to travel to the left.
     *
     * @param newVelocityY Parameter to modify
     * @return this object
     */
    public RJFieldCentricFacingPoint withVelocityY(LinearVelocity newVelocityY) {
      this.VelocityY = newVelocityY.in(MetersPerSecond);
      return this;
    }

    /**
     * Modifies the TargetPoint parameter and returns itself.
     *
     * <p>The desired point to face. (0, 0) is the left corner of the blue alliance station wall
     * when in the driver station, according to WPILib convention.
     *
     * @param newTargetPoint Parameter to modify
     * @return this object
     */
    public RJFieldCentricFacingPoint withTargetPoint(Translation2d newTargetPoint) {
      this.TargetPoint = newTargetPoint;
      return this;
    }

    /**
     * Modifies the HeadingOffset parameter and returns itself.
     *
     * <p>An offset to the face of the robot which targets the TargetPoint. Useful when the
     * non-front face of the robot should face TargetPoint.
     *
     * @param newHeadingOffset Parameter to modify
     * @return this object
     */
    public RJFieldCentricFacingPoint withHeadingOffset(Rotation2d newHeadingOffset) {
      this.HeadingOffset = newHeadingOffset;
      return this;
    }

    /**
     * Modifies the Deadband parameter and returns itself.
     *
     * <p>The allowable deadband of the request, in m/s.
     *
     * @param newDeadband Parameter to modify
     * @return this object
     */
    public RJFieldCentricFacingPoint withDeadband(double newDeadband) {
      this.Deadband = newDeadband;
      return this;
    }

    /**
     * Modifies the Deadband parameter and returns itself.
     *
     * <p>The allowable deadband of the request, in m/s.
     *
     * @param newDeadband Parameter to modify
     * @return this object
     */
    public RJFieldCentricFacingPoint withDeadband(LinearVelocity newDeadband) {
      this.Deadband = newDeadband.in(MetersPerSecond);
      return this;
    }

    /**
     * Modifies the RotationalDeadband parameter and returns itself.
     *
     * <p>The rotational deadband of the request, in radians per second.
     *
     * @param newRotationalDeadband Parameter to modify
     * @return this object
     */
    public RJFieldCentricFacingPoint withRotationalDeadband(double newRotationalDeadband) {
      this.RotationalDeadband = newRotationalDeadband;
      return this;
    }

    /**
     * Modifies the RotationalDeadband parameter and returns itself.
     *
     * <p>The rotational deadband of the request, in radians per second.
     *
     * @param newRotationalDeadband Parameter to modify
     * @return this object
     */
    public RJFieldCentricFacingPoint withRotationalDeadband(AngularVelocity newRotationalDeadband) {
      this.RotationalDeadband = newRotationalDeadband.in(RadiansPerSecond);
      return this;
    }

    /**
     * Modifies the CenterOfRotation parameter and returns itself.
     *
     * <p>The center of rotation the robot should rotate around. This is (0,0) by default, which
     * will rotate around the center of the robot.
     *
     * @param newCenterOfRotation Parameter to modify
     * @return this object
     */
    public RJFieldCentricFacingPoint withCenterOfRotation(Translation2d newCenterOfRotation) {
      this.CenterOfRotation = newCenterOfRotation;
      return this;
    }

    /**
     * Modifies the DriveRequestType parameter and returns itself.
     *
     * <p>The type of control request to use for the drive motor.
     *
     * @param newDriveRequestType Parameter to modify
     * @return this object
     */
    public RJFieldCentricFacingPoint withDriveRequestType(
        SwerveModule.DriveRequestType newDriveRequestType) {
      this.DriveRequestType = newDriveRequestType;
      return this;
    }

    /**
     * Modifies the SteerRequestType parameter and returns itself.
     *
     * <p>The type of control request to use for the drive motor.
     *
     * @param newSteerRequestType Parameter to modify
     * @return this object
     */
    public RJFieldCentricFacingPoint withSteerRequestType(
        SwerveModule.SteerRequestType newSteerRequestType) {
      this.SteerRequestType = newSteerRequestType;
      return this;
    }

    /**
     * Modifies the DesaturateWheelSpeeds parameter and returns itself.
     *
     * <p>Whether to desaturate wheel speeds before applying. For more information, see the
     * documentation of {@link SwerveDriveKinematics#desaturateWheelSpeeds}.
     *
     * @param newDesaturateWheelSpeeds Parameter to modify
     * @return this object
     */
    public RJFieldCentricFacingPoint withDesaturateWheelSpeeds(boolean newDesaturateWheelSpeeds) {
      this.DesaturateWheelSpeeds = newDesaturateWheelSpeeds;
      return this;
    }

    /**
     * Modifies the ForwardPerspective parameter and returns itself.
     *
     * <p>The perspective to use when determining which direction is forward.
     *
     * @param newForwardPerspective Parameter to modify
     * @return this object
     */
    public RJFieldCentricFacingPoint withForwardPerspective(
        ForwardPerspectiveValue newForwardPerspective) {
      this.ForwardPerspective = newForwardPerspective;
      return this;
    }
  }

  /**
   * Drives the swerve drivetrain in a robot-centric manner, maintaining a specified heading angle
   * to ensure the robot is facing the desired point on the field.
   */
  public static class RJRobotCentricFacingPoint implements SwerveRequest {
    /**
     * The velocity in the X direction, in m/s. X is defined as forward according to WPILib
     * convention, so this determines how fast to travel forward.
     */
    public double VelocityX = 0;

    /**
     * The velocity in the Y direction, in m/s. Y is defined as to the left according to WPILib
     * convention, so this determines how fast to travel to the left.
     */
    public double VelocityY = 0;

    /**
     * The desired position to face. (0, 0) is the left corner of the blue alliance station wall
     * when in the driver station, according to WPILib convention.
     */
    public Translation2d TargetPoint = new Translation2d();

    /**
     * An offset to the face of the robot which targets the TargetPoint. Useful when the non-front
     * face of the robot should face TargetPoint.
     */
    public Rotation2d HeadingOffset = new Rotation2d();

    /** The allowable deadband of the request, in m/s. */
    public double Deadband = 0;

    /** The rotational deadband of the request, in radians per second. */
    public double RotationalDeadband = 0;

    /**
     * The center of rotation the robot should rotate around. This is (0,0) by default, which will
     * rotate around the center of the robot.
     */
    public Translation2d CenterOfRotation = new Translation2d();

    /** The type of control request to use for the drive motor. */
    public SwerveModule.DriveRequestType DriveRequestType =
        SwerveModule.DriveRequestType.OpenLoopVoltage;

    /** The type of control request to use for the steer motor. */
    public SwerveModule.SteerRequestType SteerRequestType = SwerveModule.SteerRequestType.Position;

    /**
     * Whether to desaturate wheel speeds before applying. For more information, see the
     * documentation of {@link SwerveDriveKinematics#desaturateWheelSpeeds}.
     */
    public boolean DesaturateWheelSpeeds = true;

    /**
     * The PID controller used to maintain the desired heading. Users can specify the PID gains to
     * change how aggressively to maintain heading.
     *
     * <p>This PID controller operates on heading radians and outputs a target rotational rate in
     * radians per second. Note that continuous input should be enabled on the range [-pi, pi].
     */
    public PhoenixPIDController HeadingController = new PhoenixPIDController(0, 0, 0);

    private final RobotCentric m_robotCentric = new RobotCentric();

    public RJRobotCentricFacingPoint() {
      HeadingController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public StatusCode apply(
        SwerveControlParameters parameters, SwerveModule<?, ?, ?>... modulesToApply) {
      Translation2d pointToFace = TargetPoint;
      Translation2d pointOffset = pointToFace.minus(parameters.currentPose.getTranslation());
      Rotation2d headingToFace =
          pointOffset.getAngle().plus(Rotation2d.fromRotations(0.5)).plus(HeadingOffset);

      var angularVelocity = 0.0;
      var top = VelocityY * pointOffset.getX() - VelocityX * pointOffset.getY();
      var bottom =
          pointOffset.getX() * pointOffset.getX() + pointOffset.getY() * pointOffset.getY();

      if (bottom > 1e-9) {
        angularVelocity = top / bottom;
      }

      double toApplyOmega =
          -angularVelocity
              + HeadingController.calculate(
                  parameters.currentPose.getRotation().getRadians(),
                  headingToFace.getRadians(),
                  parameters.timestamp);

      ChassisSpeeds toApply =
          ChassisSpeeds.fromRobotRelativeSpeeds(VelocityX, VelocityY, toApplyOmega, HeadingOffset);

      return m_robotCentric
          .withVelocityX(toApply.vxMetersPerSecond)
          .withVelocityY(toApply.vyMetersPerSecond)
          .withRotationalRate(toApply.omegaRadiansPerSecond)
          .withDeadband(Deadband)
          .withRotationalDeadband(RotationalDeadband)
          .withCenterOfRotation(CenterOfRotation)
          .withDriveRequestType(DriveRequestType)
          .withSteerRequestType(SteerRequestType)
          .withDesaturateWheelSpeeds(DesaturateWheelSpeeds)
          .apply(parameters, modulesToApply);
    }

    /**
     * Modifies the VelocityX parameter and returns itself.
     *
     * <p>The velocity in the X direction, in m/s. X is defined as forward according to WPILib
     * convention, so this determines how fast to travel forward.
     *
     * @param newVelocityX Parameter to modify
     * @return this object
     */
    public RJRobotCentricFacingPoint withVelocityX(double newVelocityX) {
      this.VelocityX = newVelocityX;
      return this;
    }

    /**
     * Modifies the VelocityX parameter and returns itself.
     *
     * <p>The velocity in the X direction, in m/s. X is defined as forward according to WPILib
     * convention, so this determines how fast to travel forward.
     *
     * @param newVelocityX Parameter to modify
     * @return this object
     */
    public RJRobotCentricFacingPoint withVelocityX(LinearVelocity newVelocityX) {
      this.VelocityX = newVelocityX.in(MetersPerSecond);
      return this;
    }

    /**
     * Modifies the VelocityY parameter and returns itself.
     *
     * <p>The velocity in the Y direction, in m/s. Y is defined as to the left according to WPILib
     * convention, so this determines how fast to travel to the left.
     *
     * @param newVelocityY Parameter to modify
     * @return this object
     */
    public RJRobotCentricFacingPoint withVelocityY(double newVelocityY) {
      this.VelocityY = newVelocityY;
      return this;
    }

    /**
     * Modifies the VelocityY parameter and returns itself.
     *
     * <p>The velocity in the Y direction, in m/s. Y is defined as to the left according to WPILib
     * convention, so this determines how fast to travel to the left.
     *
     * @param newVelocityY Parameter to modify
     * @return this object
     */
    public RJRobotCentricFacingPoint withVelocityY(LinearVelocity newVelocityY) {
      this.VelocityY = newVelocityY.in(MetersPerSecond);
      return this;
    }

    /**
     * Modifies the TargetPoint parameter and returns itself.
     *
     * <p>The desired point to face. (0, 0) is the left corner of the blue alliance station wall
     * when in the driver station, according to WPILib convention.
     *
     * @param newTargetPoint Parameter to modify
     * @return this object
     */
    public RJRobotCentricFacingPoint withTargetPoint(Translation2d newTargetPoint) {
      this.TargetPoint = newTargetPoint;
      return this;
    }

    /**
     * Modifies the HeadingOffset parameter and returns itself.
     *
     * <p>An offset to the face of the robot which targets the TargetPoint. Useful when the
     * non-front face of the robot should face TargetPoint.
     *
     * @param newHeadingOffset Parameter to modify
     * @return this object
     */
    public RJRobotCentricFacingPoint withHeadingOffset(Rotation2d newHeadingOffset) {
      this.HeadingOffset = newHeadingOffset;
      return this;
    }

    /**
     * Modifies the Deadband parameter and returns itself.
     *
     * <p>The allowable deadband of the request, in m/s.
     *
     * @param newDeadband Parameter to modify
     * @return this object
     */
    public RJRobotCentricFacingPoint withDeadband(double newDeadband) {
      this.Deadband = newDeadband;
      return this;
    }

    /**
     * Modifies the Deadband parameter and returns itself.
     *
     * <p>The allowable deadband of the request, in m/s.
     *
     * @param newDeadband Parameter to modify
     * @return this object
     */
    public RJRobotCentricFacingPoint withDeadband(LinearVelocity newDeadband) {
      this.Deadband = newDeadband.in(MetersPerSecond);
      return this;
    }

    /**
     * Modifies the RotationalDeadband parameter and returns itself.
     *
     * <p>The rotational deadband of the request, in radians per second.
     *
     * @param newRotationalDeadband Parameter to modify
     * @return this object
     */
    public RJRobotCentricFacingPoint withRotationalDeadband(double newRotationalDeadband) {
      this.RotationalDeadband = newRotationalDeadband;
      return this;
    }

    /**
     * Modifies the RotationalDeadband parameter and returns itself.
     *
     * <p>The rotational deadband of the request, in radians per second.
     *
     * @param newRotationalDeadband Parameter to modify
     * @return this object
     */
    public RJRobotCentricFacingPoint withRotationalDeadband(AngularVelocity newRotationalDeadband) {
      this.RotationalDeadband = newRotationalDeadband.in(RadiansPerSecond);
      return this;
    }

    /**
     * Modifies the CenterOfRotation parameter and returns itself.
     *
     * <p>The center of rotation the robot should rotate around. This is (0,0) by default, which
     * will rotate around the center of the robot.
     *
     * @param newCenterOfRotation Parameter to modify
     * @return this object
     */
    public RJRobotCentricFacingPoint withCenterOfRotation(Translation2d newCenterOfRotation) {
      this.CenterOfRotation = newCenterOfRotation;
      return this;
    }

    /**
     * Modifies the DriveRequestType parameter and returns itself.
     *
     * <p>The type of control request to use for the drive motor.
     *
     * @param newDriveRequestType Parameter to modify
     * @return this object
     */
    public RJRobotCentricFacingPoint withDriveRequestType(
        SwerveModule.DriveRequestType newDriveRequestType) {
      this.DriveRequestType = newDriveRequestType;
      return this;
    }

    /**
     * Modifies the SteerRequestType parameter and returns itself.
     *
     * <p>The type of control request to use for the drive motor.
     *
     * @param newSteerRequestType Parameter to modify
     * @return this object
     */
    public RJRobotCentricFacingPoint withSteerRequestType(
        SwerveModule.SteerRequestType newSteerRequestType) {
      this.SteerRequestType = newSteerRequestType;
      return this;
    }

    /**
     * Modifies the DesaturateWheelSpeeds parameter and returns itself.
     *
     * <p>Whether to desaturate wheel speeds before applying. For more information, see the
     * documentation of {@link SwerveDriveKinematics#desaturateWheelSpeeds}.
     *
     * @param newDesaturateWheelSpeeds Parameter to modify
     * @return this object
     */
    public RJRobotCentricFacingPoint withDesaturateWheelSpeeds(boolean newDesaturateWheelSpeeds) {
      this.DesaturateWheelSpeeds = newDesaturateWheelSpeeds;
      return this;
    }
  }

  /**
   * Drives the swerve drivetrain in a robot-centric manner according to a SwerveSample from a
   * Choreo Trajectory.
   */
  public static class RJFollowTrajectory implements SwerveRequest {
    /** The trajectory sample to adhere to. */
    public SwerveSample TrajectorySample =
        new SwerveSample(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, new double[4], new double[4]);

    /**
     * The center of rotation the robot should rotate around. This is (0,0) by default, which will
     * rotate around the center of the robot.
     */
    public Translation2d CenterOfRotation = new Translation2d();

    /** The type of control request to use for the drive motor. */
    public SwerveModule.DriveRequestType DriveRequestType =
        SwerveModule.DriveRequestType.OpenLoopVoltage;

    /** The type of control request to use for the steer motor. */
    public SwerveModule.SteerRequestType SteerRequestType = SwerveModule.SteerRequestType.Position;

    /**
     * Whether to desaturate wheel speeds before applying. For more information, see the
     * documentation of {@link SwerveDriveKinematics#desaturateWheelSpeeds}.
     */
    public boolean DesaturateWheelSpeeds = true;

    /**
     * The PID controller used to maintain the sampled pose. Users can specify the PID gains to
     * change how aggressively to maintain heading.
     *
     * <p>This PID controller operates on heading radians and outputs a target rotational rate in
     * radians per second. Note that continuous input should be enabled on the range [-pi, pi].
     */
    public PhoenixPIDController XController = new PhoenixPIDController(0, 0, 0);

    public PhoenixPIDController YController = new PhoenixPIDController(0, 0, 0);
    public PhoenixPIDController HeadingController = new PhoenixPIDController(0, 0, 0);

    public CTREHolonomicController Controller =
        new CTREHolonomicController(XController, YController, HeadingController);

    private final ApplyRobotSpeeds m_robotCentric = new ApplyRobotSpeeds();

    public RJFollowTrajectory() {
      HeadingController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public StatusCode apply(
        SwerveControlParameters parameters, SwerveModule<?, ?, ?>... modulesToApply) {
      SwerveSample sampleToFollow = TrajectorySample;

      ChassisSpeeds speeds =
          Controller.calculate(
              parameters.currentPose,
              sampleToFollow.getPose(),
              sampleToFollow.getChassisSpeeds(),
              parameters.timestamp);

      return m_robotCentric
          .withSpeeds(speeds)
          .withWheelForceFeedforwardsX(sampleToFollow.moduleForcesX())
          .withWheelForceFeedforwardsY(sampleToFollow.moduleForcesY())
          .withCenterOfRotation(CenterOfRotation)
          .withDriveRequestType(DriveRequestType)
          .withSteerRequestType(SteerRequestType)
          .withDesaturateWheelSpeeds(DesaturateWheelSpeeds)
          .apply(parameters, modulesToApply);
    }

    /**
     * Modifies the TrajectorySample parameter and returns itself.
     *
     * @param newTrajectorySample Parameter to modify
     * @return this object
     */
    public RJFollowTrajectory withTrajectorySample(SwerveSample newTrajectorySample) {
      this.TrajectorySample = newTrajectorySample;
      return this;
    }

    /**
     * Modifies the CenterOfRotation parameter and returns itself.
     *
     * <p>The center of rotation the robot should rotate around. This is (0,0) by default, which
     * will rotate around the center of the robot.
     *
     * @param newCenterOfRotation Parameter to modify
     * @return this object
     */
    public RJFollowTrajectory withCenterOfRotation(Translation2d newCenterOfRotation) {
      this.CenterOfRotation = newCenterOfRotation;
      return this;
    }

    /**
     * Modifies the DriveRequestType parameter and returns itself.
     *
     * <p>The type of control request to use for the drive motor.
     *
     * @param newDriveRequestType Parameter to modify
     * @return this object
     */
    public RJFollowTrajectory withDriveRequestType(
        SwerveModule.DriveRequestType newDriveRequestType) {
      this.DriveRequestType = newDriveRequestType;
      return this;
    }

    /**
     * Modifies the SteerRequestType parameter and returns itself.
     *
     * <p>The type of control request to use for the drive motor.
     *
     * @param newSteerRequestType Parameter to modify
     * @return this object
     */
    public RJFollowTrajectory withSteerRequestType(
        SwerveModule.SteerRequestType newSteerRequestType) {
      this.SteerRequestType = newSteerRequestType;
      return this;
    }

    /**
     * Modifies the DesaturateWheelSpeeds parameter and returns itself.
     *
     * <p>Whether to desaturate wheel speeds before applying. For more information, see the
     * documentation of {@link SwerveDriveKinematics#desaturateWheelSpeeds}.
     *
     * @param newDesaturateWheelSpeeds Parameter to modify
     * @return this object
     */
    public RJFollowTrajectory withDesaturateWheelSpeeds(boolean newDesaturateWheelSpeeds) {
      this.DesaturateWheelSpeeds = newDesaturateWheelSpeeds;
      return this;
    }
  }

  /** Accepts a Pose and robot-centric ChassisSpeeds to apply to the drivetrain. */
  public static class RJHoldState implements SwerveRequest {
    /** The pose to hold the drivetrain to. */
    public Pose2d TargetPose = new Pose2d();

    /** The robot-centric chassis speeds to apply to the drivetrain. */
    public ChassisSpeeds Speeds = new ChassisSpeeds();

    /**
     * The center of rotation the robot should rotate around. This is (0,0) by default, which will
     * rotate around the center of the robot.
     */
    public Translation2d CenterOfRotation = new Translation2d();

    /** The type of control request to use for the drive motor. */
    public SwerveModule.DriveRequestType DriveRequestType =
        SwerveModule.DriveRequestType.OpenLoopVoltage;

    /** The type of control request to use for the steer motor. */
    public SwerveModule.SteerRequestType SteerRequestType = SwerveModule.SteerRequestType.Position;

    /**
     * Whether to desaturate wheel speeds before applying. For more information, see the
     * documentation of {@link SwerveDriveKinematics#desaturateWheelSpeeds}.
     */
    public boolean DesaturateWheelSpeeds = true;

    /**
     * The PID controller used to maintain the sampled pose. Users can specify the PID gains to
     * change how aggressively to maintain heading.
     *
     * <p>This PID controller operates on heading radians and outputs a target rotational rate in
     * radians per second. Note that continuous input should be enabled on the range [-pi, pi].
     */
    public PhoenixPIDController XController = new PhoenixPIDController(0, 0, 0);

    public PhoenixPIDController YController = new PhoenixPIDController(0, 0, 0);
    public PhoenixPIDController HeadingController = new PhoenixPIDController(0, 0, 0);

    public CTREHolonomicController Controller =
        new CTREHolonomicController(XController, YController, HeadingController);

    private final ApplyRobotSpeeds m_robotCentric = new ApplyRobotSpeeds();

    public RJHoldState() {
      HeadingController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public StatusCode apply(
        SwerveControlParameters parameters, SwerveModule<?, ?, ?>... modulesToApply) {
      ChassisSpeeds toApply =
          Controller.calculate(parameters.currentPose, TargetPose, Speeds, parameters.timestamp);

      if (Math.abs(toApply.vxMetersPerSecond) < 0.02) {
        toApply.vxMetersPerSecond = 0;
      }

      if (Math.abs(toApply.vyMetersPerSecond) < 0.02) {
        toApply.vyMetersPerSecond = 0;
      }

      if (Math.abs(toApply.omegaRadiansPerSecond) < 0.02) {
        toApply.omegaRadiansPerSecond = 0;
      }

      return m_robotCentric
          .withSpeeds(toApply)
          .withCenterOfRotation(CenterOfRotation)
          .withDriveRequestType(DriveRequestType)
          .withSteerRequestType(SteerRequestType)
          .withDesaturateWheelSpeeds(DesaturateWheelSpeeds)
          .apply(parameters, modulesToApply);
    }

    /**
     * Modifies the TargetPose parameter and returns itself.
     *
     * @param newTargetPose Parameter to modify
     * @return this object
     */
    public RJHoldState withTargetPose(Pose2d newTargetPose) {
      this.TargetPose = newTargetPose;
      return this;
    }

    /**
     * Modifies the Speeds parameter and returns itself.
     *
     * @param newSpeeds Parameter to modify
     * @return this object
     */
    public RJHoldState withSpeeds(ChassisSpeeds newSpeeds) {
      this.Speeds = newSpeeds;
      return this;
    }

    /**
     * Modifies the CenterOfRotation parameter and returns itself.
     *
     * <p>The center of rotation the robot should rotate around. This is (0,0) by default, which
     * will rotate around the center of the robot.
     *
     * @param newCenterOfRotation Parameter to modify
     * @return this object
     */
    public RJHoldState withCenterOfRotation(Translation2d newCenterOfRotation) {
      this.CenterOfRotation = newCenterOfRotation;
      return this;
    }

    /**
     * Modifies the DriveRequestType parameter and returns itself.
     *
     * <p>The type of control request to use for the drive motor.
     *
     * @param newDriveRequestType Parameter to modify
     * @return this object
     */
    public RJHoldState withDriveRequestType(SwerveModule.DriveRequestType newDriveRequestType) {
      this.DriveRequestType = newDriveRequestType;
      return this;
    }

    /**
     * Modifies the SteerRequestType parameter and returns itself.
     *
     * <p>The type of control request to use for the drive motor.
     *
     * @param newSteerRequestType Parameter to modify
     * @return this object
     */
    public RJHoldState withSteerRequestType(SwerveModule.SteerRequestType newSteerRequestType) {
      this.SteerRequestType = newSteerRequestType;
      return this;
    }

    /**
     * Modifies the DesaturateWheelSpeeds parameter and returns itself.
     *
     * <p>Whether to desaturate wheel speeds before applying. For more information, see the
     * documentation of {@link SwerveDriveKinematics#desaturateWheelSpeeds}.
     *
     * @param newDesaturateWheelSpeeds Parameter to modify
     * @return this object
     */
    public RJHoldState withDesaturateWheelSpeeds(boolean newDesaturateWheelSpeeds) {
      this.DesaturateWheelSpeeds = newDesaturateWheelSpeeds;
      return this;
    }
  }
}
