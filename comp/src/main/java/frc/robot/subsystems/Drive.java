package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.wpilibj2.command.Commands.either;
import static edu.wpi.first.wpilibj2.command.Commands.none;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static frc.robot.oi.HeadingTarget.EAST;
import static frc.robot.oi.HeadingTarget.INTAKE;
import static frc.robot.oi.HeadingTarget.NONE;
import static frc.robot.oi.HeadingTarget.WEST;
import static frc.robot.oi.StructureState.SCORE_ALGAE_BARGE;
import static frc.robot.oi.StructureState.SCORE_ALGAE_PROCESSOR;
import static java.lang.Math.abs;

import choreo.trajectory.SwerveSample;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ApplyFieldSpeeds;
import com.ctre.phoenix6.swerve.SwerveRequest.ApplyRobotSpeeds;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentricFacingAngle;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.ctre.phoenix6.swerve.SwerveRequest.Idle;
import com.ctre.phoenix6.swerve.SwerveRequest.RobotCentric;
import com.ctre.phoenix6.swerve.SwerveRequest.RobotCentricFacingAngle;
import com.ctre.phoenix6.swerve.SwerveRequest.SwerveDriveBrake;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathfindThenFollowPath;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.AutoAlignToBranch;
import frc.robot.constants.Constants.ControlsConfig;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.FieldConstants.Side;
import frc.robot.constants.FieldGeometry;
import frc.robot.constants.TunerConstants;
import frc.robot.constants.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.ctre.RJSwerveRequest.RJFieldCentricFacingPoint;
import frc.robot.ctre.RJSwerveRequest.RJFollowTrajectory;
import frc.robot.ctre.RJSwerveRequest.RJHoldState;
import frc.robot.ctre.RJSwerveRequest.RJRobotCentricFacingPoint;
import frc.robot.lib.AllianceFlipUtil;
import frc.robot.lib.RJLog;
import frc.robot.lib.pathplanner.PPHolonomicDriveController;
import frc.robot.lib.pathplanning.RepulsorFieldPlanner;
import frc.robot.oi.ControlMode;
import frc.robot.oi.DriverControls;
import frc.robot.oi.HeadingTarget;
import frc.robot.oi.StructureState;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Function;
import java.util.function.Supplier;

public class Drive extends TunerSwerveDrivetrain implements Subsystem {
  private static final double kSimLoopPeriod = 0.005; // 5 ms
  private Notifier m_simNotifier = null;
  private double m_lastSimTime;

  private SwerveRequest m_requestToApply;

  /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
  private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.fromDegrees(0);
  /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
  private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.fromDegrees(180);
  /* Keep track if we've ever applied the operator perspective before or not */
  private boolean m_hasAppliedOperatorPerspective = false;
  private Alliance alliance = Alliance.Blue;

  /* Swerve requests to apply during SysId characterization */
  private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization =
      new SwerveRequest.SysIdSwerveTranslation();
  private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization =
      new SwerveRequest.SysIdSwerveSteerGains();
  private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization =
      new SwerveRequest.SysIdSwerveRotation();

  private final SwerveDriveBrake brake = new SwerveDriveBrake();
  private final Idle idle = new Idle();

  private final FieldCentric fieldCentric = new FieldCentric();
  private final RobotCentric robotCentric = new RobotCentric();

  private final FieldCentricFacingAngle fieldCentricFacingAngle = new FieldCentricFacingAngle();
  private final RobotCentricFacingAngle robotCentricFacingAngle = new RobotCentricFacingAngle();

  private final RJFieldCentricFacingPoint fieldCentricFacingPoint =
      new RJFieldCentricFacingPoint().withForwardPerspective(ForwardPerspectiveValue.BlueAlliance);
  private final RJRobotCentricFacingPoint robotCentricFacingPoint = new RJRobotCentricFacingPoint();

  private final RJFollowTrajectory followTrajectory =
      new RJFollowTrajectory().withDriveRequestType(DriveRequestType.Velocity);

  private final RobotConfig ppRobotConfig = loadRobotConfig();
  private final PathConstraints ppConstraints =
      new PathConstraints(
          MetersPerSecond.of(4),
          MetersPerSecondPerSecond.of(5),
          RotationsPerSecond.of(1),
          RotationsPerSecondPerSecond.of(2));

  /*
   * Swerve request to apply during path following:
   * Pathplanner uses robot-centric speeds,
   * Choreo uses field-centric speeds
   */
  private final ApplyRobotSpeeds applyRobotSpeeds =
      new ApplyRobotSpeeds().withDriveRequestType(DriveRequestType.Velocity);

  private final ApplyFieldSpeeds applyFieldSpeeds =
      new ApplyFieldSpeeds().withDriveRequestType(DriveRequestType.Velocity);

  private final PIDController pathXController = new PIDController(0, 0, 0);
  private final PIDController pathYController = new PIDController(0, 0, 0);
  private final PIDController pathThetaController = new PIDController(0, 0, 0);

  private final PIDConstants translationConstants = new PIDConstants(3);
  private final PIDConstants rotationConstants = new PIDConstants(10);

  private final RJHoldState holdState =
      new RJHoldState().withDriveRequestType(DriveRequestType.Velocity);

  /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
  private final SysIdRoutine m_sysIdRoutineTranslation =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null, // Use default ramp rate (1 V/s)
              Volts.of(7), // Reduce dynamic step voltage to 4 V to prevent brownout
              null, // Use default timeout (10 s)
              // Log state with SignalLogger class
              state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())),
          new SysIdRoutine.Mechanism(
              output -> setControl(m_translationCharacterization.withVolts(output)), null, this));

  /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
  private final SysIdRoutine m_sysIdRoutineSteer =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null, // Use default ramp rate (1 V/s)
              Volts.of(7), // Use dynamic voltage of 7 V
              null, // Use default timeout (10 s)
              // Log state with SignalLogger class
              state -> SignalLogger.writeString("SysIdSteer_State", state.toString())),
          new SysIdRoutine.Mechanism(
              volts -> setControl(m_steerCharacterization.withVolts(volts)), null, this));

  /*
   * SysId routine for characterizing rotation.
   * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
   * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
   */
  private final SysIdRoutine m_sysIdRoutineRotation =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              /* This is in radians per second², but SysId only supports "volts per second" */
              Volts.of(Math.PI / 6).per(Second),
              /* This is in radians per second, but SysId only supports "volts" */
              Volts.of(Math.PI),
              null, // Use default timeout (10 s)
              // Log state with SignalLogger class
              state -> SignalLogger.writeString("SysIdRotation_State", state.toString())),
          new SysIdRoutine.Mechanism(
              output -> {
                /* output is actually radians per second, but SysId only supports "volts" */
                setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                /* also log the requested output for SysId */
                SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
              },
              null,
              this));

  /* The SysId routine to test */
  private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

  private HeadingTarget headingTarget = HeadingTarget.NONE;

  private final Alert pigeonAlert = new Alert("Pigeon is Degraded", AlertType.kError);

  private final Alert[] moduleAlerts =
      new Alert[] {
        new Alert("Front Left Module is Degraded", AlertType.kError),
        new Alert("Front Right Module is Degraded", AlertType.kError),
        new Alert("Back Left Module is Degraded", AlertType.kError),
        new Alert("Back Right Module is Degraded", AlertType.kError),
      };

  public Drive(
      SwerveDrivetrainConstants drivetrainConstants,
      double odometryUpdateFrequency,
      SwerveModuleConstants<?, ?, ?>... modules) {
    super(
        drivetrainConstants,
        odometryUpdateFrequency,
        VecBuilder.fill(0.1, 0.1, 0.1),
        VecBuilder.fill(0.1, 0.1, 0.1),
        modules);

    if (Utils.isSimulation()) {
      startSimThread();
    }

    configureRotationPID(robotCentricFacingAngle.HeadingController);
    configureRotationPID(fieldCentricFacingAngle.HeadingController);
    configureRotationPID(fieldCentricFacingPoint.HeadingController);

    configureTranslationPID(followTrajectory.XController);
    configureTranslationPID(followTrajectory.YController);
    configureRotationPID(followTrajectory.HeadingController);

    configureTranslationPID(holdState.XController);
    configureTranslationPID(holdState.YController);
    configureRotationPID(holdState.HeadingController);

    pathThetaController.enableContinuousInput(-Math.PI, Math.PI);

    configureTranslationPID(pathXController);
    configureTranslationPID(pathYController);
    configureRotationPID(pathThetaController);

    configureAutoBuilder();

    pathThetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  private RobotConfig loadRobotConfig() {
    try {
      return RobotConfig.fromGUISettings();
    } catch (Exception ex) {
      DriverStation.reportError("Failed to load PathPlanner config", ex.getStackTrace());
    }

    return new RobotConfig(
        Kilograms.of(65.7),
        KilogramSquareMeters.of(6),
        new ModuleConfig(
            Inches.of(1.75),
            MetersPerSecond.of(4.5),
            1.5,
            DCMotor.getKrakenX60(1).withReduction(5.625),
            Amps.of(80),
            1),
        new Translation2d(Inches.of(11), Inches.of(11)),
        new Translation2d(Inches.of(11), Inches.of(-11)),
        new Translation2d(Inches.of(-11), Inches.of(11)),
        new Translation2d(Inches.of(-11), Inches.of(-11)));
  }

  private void configureAutoBuilder() {
    try {
      AutoBuilder.configure(
          () -> getState().Pose, // Supplier of current robot pose
          this::resetPose, // Consumer for seeding pose against auto
          () -> getState().Speeds, // Supplier of current robot speeds
          // Consumer of ChassisSpeeds and feedforwards to drive the robot
          (speeds, feedforwards) ->
              setControl(
                  applyRobotSpeeds
                      .withSpeeds(speeds)
                      .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                      .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())),
          new PPHolonomicDriveController(
              // PID constants for translation
              translationConstants,
              // PID constants for rotation
              rotationConstants),
          ppRobotConfig,
          // Assume the path needs to be flipped for Red vs Blue, this is normally the case
          () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
          this // Subsystem for requirements
          );
    } catch (Exception ex) {
      DriverStation.reportError(
          "Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
    }
  }

  private void configureTranslationPID(PhoenixPIDController controller) {
    controller.setP(translationConstants.kP);
    controller.setI(translationConstants.kI);
    controller.setD(translationConstants.kD);
    controller.setIZone(translationConstants.iZone);

    controller.setTolerance(0.0);
  }

  private void configureTranslationPID(ProfiledPIDController controller) {
    controller.setP(translationConstants.kP);
    controller.setI(translationConstants.kI);
    controller.setD(translationConstants.kD);
    controller.setIZone(translationConstants.iZone);

    controller.setTolerance(0.0);
  }

  private void configureTranslationPID(PIDController controller) {
    controller.setP(translationConstants.kP);
    controller.setI(translationConstants.kI);
    controller.setD(translationConstants.kD);
    controller.setIZone(translationConstants.iZone);

    controller.setTolerance(0.0);
  }

  private void configureRotationPID(PhoenixPIDController controller) {
    controller.setP(rotationConstants.kP);
    controller.setI(rotationConstants.kI);
    controller.setD(rotationConstants.kD);
    controller.setIZone(rotationConstants.iZone);

    controller.setTolerance(Units.degreesToRadians(15));
  }

  private void configureRotationPID(ProfiledPIDController controller) {
    controller.setP(rotationConstants.kP);
    controller.setI(rotationConstants.kI);
    controller.setD(rotationConstants.kD);
    controller.setIZone(rotationConstants.iZone);

    controller.setTolerance(Units.degreesToRadians(15));
  }

  private void configureRotationPID(PIDController controller) {
    controller.setP(rotationConstants.kP);
    controller.setI(rotationConstants.kI);
    controller.setD(rotationConstants.kD);
    controller.setIZone(rotationConstants.iZone);

    controller.setTolerance(Units.degreesToRadians(15));
  }

  public void setHeadingTarget(HeadingTarget target) {
    this.headingTarget = target;
  }

  public Command setHeadingTargetCommand(HeadingTarget target) {
    return Commands.runOnce(() -> setHeadingTarget(target))
        .withName(String.format("Drive::setHeadingTarget(%s)", target));
  }

  public void engageBrakes() {
    configNeutralMode(NeutralModeValue.Brake);
  }

  public void disengageBrakes() {
    configNeutralMode(NeutralModeValue.Coast);
  }

  @Override
  public void setControl(SwerveRequest request) {
    m_requestToApply = request;

    super.setControl(m_requestToApply);
  }

  /**
   * Returns a command that applies the specified control request to this swerve drivetrain.
   *
   * @param request Function returning the request to apply
   * @return Command to run
   */
  public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
    return run(() -> setControl(requestSupplier.get())).withName("Drive::applyRequest");
  }

  public Command stopCommand() {
    return applyRequest(() -> brake).ignoringDisable(true).withName("Drivetrain::Stop");
  }

  public Command fieldCentricCommand(Supplier<ChassisSpeeds> fieldCentricInput) {
    return applyRequest(
            () -> {
              var input = fieldCentricInput.get();

              return fieldCentric
                  .withVelocityX(input.vx)
                  .withVelocityY(input.vy)
                  .withRotationalRate(input.omega);
            })
        .withName("Drive::FieldCentric");
  }

  public Command robotCentricCommand(Supplier<ChassisSpeeds> robotCentricInput) {
    return applyRequest(
            () -> {
              var input = robotCentricInput.get();

              return robotCentric
                  .withVelocityX(input.vx)
                  .withVelocityY(input.vy)
                  .withRotationalRate(input.omega);
            })
        .withName("Drive::RobotCentric");
  }

  public Command fieldCentricFacingAngleCommand(
      Supplier<ChassisSpeeds> fieldCentricInput, Supplier<Rotation2d> heading) {
    return applyRequest(
            () -> {
              var input = fieldCentricInput.get();

              return fieldCentricFacingAngle
                  .withVelocityX(input.vx)
                  .withVelocityY(input.vy)
                  .withTargetDirection(heading.get());
            })
        .withName("Drive::FieldCentricFacingAngle");
  }

  public Command robotCentricFacingAngleCommand(
      Supplier<ChassisSpeeds> robotCentricInput, Supplier<Rotation2d> heading) {
    return applyRequest(
            () -> {
              var input = robotCentricInput.get();

              return robotCentricFacingAngle
                  .withVelocityX(input.vx)
                  .withVelocityY(input.vy)
                  .withTargetDirection(heading.get());
            })
        .withName("Drive::RobotCentricFacingAngle");
  }

  public Command fieldCentricFacingTargetCommand(
      Supplier<ChassisSpeeds> fieldCentricInput, Supplier<Translation2d> target) {
    return applyRequest(
            () -> {
              var input = fieldCentricInput.get();

              return fieldCentricFacingPoint
                  .withVelocityX(input.vx)
                  .withVelocityY(input.vy)
                  .withTargetPoint(target.get());
            })
        .withName("Drive::FieldCentricFacingTarget");
  }

  public Command robotCentricFacingTargetCommand(
      Supplier<ChassisSpeeds> robotCentricInput, Supplier<Translation2d> target) {
    return applyRequest(
            () -> {
              var input = robotCentricInput.get();

              return robotCentricFacingPoint
                  .withVelocityX(input.vx)
                  .withVelocityY(input.vy)
                  .withTargetPoint(target.get());
            })
        .withName("Drive::RobotCentricFacingTarget");
  }

  public void followPath(SwerveSample sample) {
    var pose = getStateCopy().Pose;
    var targetSpeeds = sample.getChassisSpeeds();
    targetSpeeds.vx += pathXController.calculate(pose.getX(), sample.x);
    targetSpeeds.vy += pathYController.calculate(pose.getY(), sample.y);
    targetSpeeds.omega +=
        pathThetaController.calculate(pose.getRotation().getRadians(), sample.heading);

    RJLog.log(
        "Drive/Path/Error", sample.getPose().getTranslation().getDistance(pose.getTranslation()));
    RJLog.log("Drive/Path/Target", sample.getPose());
    RJLog.log("Drive/Path/Speeds", sample.getChassisSpeeds());
    RJLog.log(
        "Drive/Path/MeasuredSpeeds", getStateCopy().Speeds.toFieldRelative(pose.getRotation()));
    RJLog.log("Drive/Path/SpeedsWithPID", targetSpeeds);

    setControl(
        applyFieldSpeeds
            .withSpeeds(targetSpeeds)
            .withWheelForceFeedforwardsX(sample.moduleForcesX())
            .withWheelForceFeedforwardsY(sample.moduleForcesY()));
  }

  public Command HoldState(Pose2d pose, ChassisSpeeds speeds) {
    return applyRequest(() -> holdState.withTargetPose(pose).withSpeeds(speeds))
        .withName("Drive::HoldState");
  }

  public Command LockWheels() {
    return applyRequest(() -> brake).withName("Drive::LockWheels");
  }

  public Command teleopCommand(
      DriverControls driver,
      Supplier<ControlMode> controlModeSupplier,
      Supplier<StructureState> targetState,
      BooleanSupplier visionEnabled) {
    return applyRequest(
            () ->
                teleop(
                    driver,
                    controlModeSupplier.get(),
                    targetState.get(),
                    visionEnabled.getAsBoolean()))
        .withName("Drive::Teleop");
  }

  public SwerveRequest teleop(
      DriverControls driver,
      ControlMode controlMode,
      StructureState currentTarget,
      boolean visionEnabled) {
    // Heading snaps
    switch (controlMode) {
      case CYCLING_MODE:
        if (visionEnabled && driver.intake()) {
          headingTarget = INTAKE;
        } else if (driver.aim()) {
          if (currentTarget == SCORE_ALGAE_BARGE) {
            // headingTarget = NORTH;
          } else if (currentTarget == SCORE_ALGAE_PROCESSOR) {
            if (isDownfield()) {
              headingTarget = WEST;
            } else {
              headingTarget = EAST;
            }
          }
        } else {
          headingTarget = NONE;
        }
        break;
      case CLIMB_MODE:
        // headingTarget = WEST;
    }

    if (headingTarget != NONE) {
      targetHeading = getStateCopy().Pose.getRotation();
      RJLog.log("DriveMode", "HeadingSnap");
      return headingSnap(driver, controlMode, headingTarget);
    }

    if (driver.config.PathToAnyFace()) {
      if (driver.shouldManuallyRotate()) {
        targetHeading = getStateCopy().Pose.getRotation();
        RJLog.log("DriveMode", "Manual");

        return manualRotation(driver, controlMode);
      }

      RJLog.log("DriveMode", "PathToFace");

      return faceHeading(driver, controlMode, targetHeading);
    }

    RJLog.log("DriveMode", "Manual2");

    return manualRotation(driver, controlMode);
  }

  private Rotation2d targetHeading = new Rotation2d();

  public SwerveRequest manualRotation(DriverControls driver, ControlMode controlMode) {
    if (driver.shouldUseRobotCentric()) {
      var input = driver.robotCentric(controlMode);

      RJLog.log("Driver/RobotCentricInput", input);

      return robotCentric
          .withVelocityX(input.vx)
          .withVelocityY(input.vy)
          .withRotationalRate(input.omega);
    }

    var input = driver.fieldCentric(controlMode);

    RJLog.log("Driver/FieldCentricInput", input);

    return fieldCentric
        .withVelocityX(input.vx)
        .withVelocityY(input.vy)
        .withRotationalRate(input.omega);
  }

  public SwerveRequest headingSnap(
      DriverControls driver, ControlMode controlMode, HeadingTarget target) {
    switch (target) {
      case INTAKE:
      case FORWARD:
      case BACKWARD:
      case LEFT:
      case RIGHT:
      case NORTH:
      case SOUTH:
      case EAST:
      case WEST:
        return faceHeading(driver, controlMode, target.getHeading(alliance, getStateCopy().Pose));

      case NONE:
      default:
        break;
    }

    return manualRotation(driver, controlMode);
  }

  public SwerveRequest faceHeading(
      DriverControls driver, ControlMode controlMode, Rotation2d target) {
    target = AllianceFlipUtil.rotate(target);
    if (driver.shouldUseRobotCentric()) {
      var input = driver.robotCentric(controlMode);
      return robotCentricFacingAngle
          .withVelocityX(input.vx)
          .withVelocityY(input.vy)
          .withTargetDirection(target);
    }

    var input = driver.fieldCentric(controlMode);

    return fieldCentricFacingAngle
        .withVelocityX(input.vx)
        .withVelocityY(input.vy)
        .withTargetDirection(target);
  }

  public SwerveRequest faceTarget(
      DriverControls driver, ControlMode controlMode, Translation2d target) {
    if (driver.shouldUseRobotCentric()) {
      var input = driver.robotCentric(controlMode);

      return robotCentricFacingPoint
          .withVelocityX(input.vx)
          .withVelocityY(input.vy)
          .withTargetPoint(target);
    }

    var input = driver.fieldCentric(controlMode);

    return fieldCentricFacingPoint
        .withVelocityX(input.vx)
        .withVelocityY(input.vy)
        .withTargetPoint(target);
  }

  /**
   * Runs the SysId Quasistatic test in the given direction for the routine specified by {@link
   * #m_sysIdRoutineToApply}.
   *
   * @param direction Direction of the SysId Quasistatic test
   * @return Command to run
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineToApply.quasistatic(direction).withName("Drive::sysidQuasistatic");
  }

  /**
   * Runs the SysId Dynamic test in the given direction for the routine specified by {@link
   * #m_sysIdRoutineToApply}.
   *
   * @param direction Direction of the SysId Dynamic test
   * @return Command to run
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineToApply.dynamic(direction).withName("Drive::sysidDynamic");
  }

  public Distance distanceTo(Translation2d point) {
    return Meters.of(getStateCopy().Pose.getTranslation().getDistance(point));
  }

  public Command resetYawCommand() {
    return Commands.runOnce(this::resetYaw).withName("Drive::ResetYaw");
  }

  public void resetYaw() {
    setOperatorPerspectiveForward(getStateCopy().Pose.getRotation());
  }

  public boolean isReady(Angle angularTolerance) {
    return isReady(Meters.of(Double.MAX_VALUE), angularTolerance);
  }

  public boolean isReady(Distance linearTolerance, Angle angularTolerance) {
    if (m_requestToApply == fieldCentric) {
      return true;
    } else if (m_requestToApply == robotCentric) {
      return true;
    } else if (m_requestToApply == brake) {
      return true;
    } else if (m_requestToApply == fieldCentricFacingAngle) {
      return fieldCentricFacingAngle.HeadingController.getPositionError()
          < angularTolerance.in(Radians);
    } else if (m_requestToApply == robotCentricFacingAngle) {
      return robotCentricFacingAngle.HeadingController.getPositionError()
          < angularTolerance.in(Radians);
    } else if (m_requestToApply == holdState) {
      holdState.Controller.setTolerance(
          new Pose2d(linearTolerance, linearTolerance, new Rotation2d(angularTolerance)));
      return holdState.Controller.atReference();
    } else if (m_requestToApply == followTrajectory) {
      followTrajectory.Controller.setTolerance(
          new Pose2d(linearTolerance, linearTolerance, new Rotation2d(angularTolerance)));
      return followTrajectory.Controller.atReference();
    } else if (m_requestToApply == fieldCentricFacingPoint) {
      return fieldCentricFacingPoint.HeadingController.getPositionError()
          < angularTolerance.in(Radians);
    } else if (m_requestToApply == robotCentricFacingPoint) {
      return robotCentricFacingPoint.HeadingController.getPositionError()
          < angularTolerance.in(Radians);
    }

    return false;
  }

  @Override
  public void periodic() {
    if (!getDefaultCommand().isScheduled()) {
      targetHeading = getStateCopy().Pose.getRotation();
    }

    RJLog.log("Drive/HeadingTarget", targetHeading);

    /*
     * Periodically try to apply the operator perspective.
     * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
     * This allows us to correct the perspective in case the robot code restarts mid-match.
     * Otherwise, only check and apply the operator perspective if the DS is disabled.
     * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
     */
    if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
      DriverStation.getAlliance()
          .ifPresent(
              allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? kRedAlliancePerspectiveRotation
                        : kBlueAlliancePerspectiveRotation);
                m_hasAppliedOperatorPerspective = true;
                alliance = allianceColor;
              });
    }

    // RJLog.log("Swerve/State", getStateCopy());
  }

  public void refreshAlert() {
    for (int i = 0; i < getModules().length; i++) {
      var module = getModule(i);
      var driveMotor = module.getDriveMotor();
      var driveHealthy = driveMotor.isConnected(); // && driveMotor.getFaultField().getValue() == 0;

      var steerMotor = module.getSteerMotor();
      var steerHealthy = steerMotor.isConnected(); // && steerMotor.getFaultField().getValue() == 0;

      moduleAlerts[i].set(!(driveHealthy && steerHealthy));
    }

    var pigeon = getPigeon2();
    var pigeonHealthy = pigeon.isConnected() && pigeon.getFaultField().getValue() == 0;

    pigeonAlert.set(!pigeonHealthy);
  }

  @Override
  public void addVisionMeasurement(
      Pose2d estimate, double time, Matrix<N3, N1> visionMeasurementStdDevs) {
    super.addVisionMeasurement(estimate, Utils.fpgaToCurrentTime(time), visionMeasurementStdDevs);
  }

  private void startSimThread() {
    m_lastSimTime = Utils.getCurrentTimeSeconds();

    /* Run simulation at a faster rate so PID gains behave more reasonably */
    m_simNotifier =
        new Notifier(
            () -> {
              final double currentTime = Utils.getCurrentTimeSeconds();
              double deltaTime = currentTime - m_lastSimTime;
              m_lastSimTime = currentTime;

              /* use the measured time delta, get battery voltage from WPILib */
              updateSimState(deltaTime, RobotController.getBatteryVoltage());
            });
    m_simNotifier.startPeriodic(kSimLoopPeriod);
  }

  public Command autoAlignToBranchCommand(Side side) {

    var autoAlignCommand =
        new AutoAlignToBranch(
            this,
            side,
            new TrapezoidProfile.Constraints(2, 2), // meters per second and meters per second sq
            new TrapezoidProfile.Constraints(2, 2), // meters per second and meters per second sq
            new TrapezoidProfile.Constraints(
                Units.degreesToRadians(180),
                Units.degreesToRadians(180)), // degrees per second and degrees per second sq
            String.format("Drive::AutoAlignToBranch(%s)", side));

    configureTranslationPID(autoAlignCommand.xController);
    configureTranslationPID(autoAlignCommand.yController);
    configureRotationPID(autoAlignCommand.rotationController);

    return autoAlignCommand.withName(String.format("Drive::AutoAlignToBranch(%s)", side));
  }

  private Pose2d getClosestBranch(Side side, Pose2d currentPose) {
    var closestTag =
        FieldConstants.getInstance()
            .getClosestReefTag(currentPose, DriverStation.getAlliance().get());
    return FieldConstants.getInstance().RobotPose(closestTag, side);
  }

  public Command pathToBranchCommand(ControlsConfig config, Side side) {
    return defer(
            () ->
                pathToBranchCommand(
                    config,
                    side,
                    FieldConstants.getInstance()
                        .getClosestReefTag(getStateCopy().Pose, DriverStation.getAlliance().get())))
        .withName(String.format("Drive::PathToNearestBranch(%s)", side));
  }

  public Command pathToBranchCommand(ControlsConfig config, Side side, int tag) {
    return either(
            pathToBranchRepulsor(side, tag),
            pathToBranchPathfindCommand(side, tag),
            config::UseForceField)
        .withName(String.format("Drive::PathToBranch(%s, %s)", side, tag));
  }

  public boolean isDownfield() {
    // blue alliance wall is at x = 0
    // red alliance wall is at x = fieldLength
    // barge is at x = fieldLength / 2
    // If our pose is past the barge and we are red, return false
    // If our pose is past the barge and we are blue, return true
    // If our pose is before the barge and we are red, return true
    // If our pose is before the barge and we are blue, return false
    // ergo, the condition is (past the barge) xor (alliance == red)
    return getStateCopy().Pose.getX() > FieldGeometry.fieldLength / 2
        ^ DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
  }

  public Command pathToBranchPathfindCommand(Side side, int tag) {
    if (tag == 0) {
      return none();
    }
    var targetBranch = FieldConstants.getInstance().RobotPose(tag, side);
    var backoff =
        new Transform2d(new Translation2d(Inches.of(-20), Inches.zero()), Rotation2d.kZero);
    var pathfindTarget = targetBranch.plus(backoff);

    var waypoints = PathPlannerPath.waypointsFromPoses(pathfindTarget, targetBranch);
    var path =
        new PathPlannerPath(
            waypoints,
            ppConstraints,
            new IdealStartingState(1, pathfindTarget.getRotation()),
            new GoalEndState(0, targetBranch.getRotation()));

    return new PathfindThenFollowPath(
            path,
            ppConstraints,
            () -> getStateCopy().Pose,
            () -> getStateCopy().Speeds,
            (speeds, feedforwards) ->
                setControl(
                    applyRobotSpeeds
                        .withSpeeds(speeds)
                        .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                        .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())),
            new PPHolonomicDriveController(translationConstants, rotationConstants),
            ppRobotConfig,
            () -> false,
            this)
        .andThen(HoldState(targetBranch, new ChassisSpeeds()))
        .withName(String.format("Drive::PathToBranch(%s)", side));
  }

  public Command pathToPoseCommand(Pose2d target) {
    return AutoBuilder.pathfindToPose(target, ppConstraints)
        .withName(String.format("Drive::PathToPose(%s)", target));
  }

  public Command pathToLeftCageCommand() {
    return defer(
            () -> {
              var target =
                  AllianceFlipUtil.rotate(
                      FieldConstants.getInstance()
                          .getTargetPoseForCage(FieldGeometry.Barge.farCage));
              return pathToPoseRepulsorCommand(target, state -> target.getRotation());
            })
        .withName("Drive::PathToLeftCage");
  }

  public Command pathToMiddleCageCommand() {
    return defer(
            () -> {
              var target =
                  AllianceFlipUtil.rotate(
                      FieldConstants.getInstance()
                          .getTargetPoseForCage(FieldGeometry.Barge.middleCage));
              return pathToPoseRepulsorCommand(target, state -> target.getRotation());
            })
        .withName("Drive::PathToMiddleCage");
  }

  public Command pathToRightCageCommand() {
    return defer(
            () -> {
              var target =
                  AllianceFlipUtil.rotate(
                      FieldConstants.getInstance()
                          .getTargetPoseForCage(FieldGeometry.Barge.closeCage));
              return pathToPoseRepulsorCommand(target, state -> target.getRotation());
            })
        .withName("Drive::PathToRightCage");
  }

  public Command pathToLeftBargeCommand() {
    return defer(
            () -> {
              var target =
                  AllianceFlipUtil.rotate(
                      AllianceFlipUtil.mirror(
                          FieldConstants.getInstance()
                              .getTargetPoseForBarge(FieldGeometry.Barge.farCage)
                              .plus(
                                  new Transform2d(
                                      new Translation2d(), Rotation2d.fromDegrees(-15))),
                          isDownfield()));
              return pathToPoseRepulsorCommand(target, state -> target.getRotation());
            })
        .withName("Drive::PathToLeftBarge");
  }

  public Command pathToRightBargeCommand() {
    return defer(
            () -> {
              var target =
                  AllianceFlipUtil.rotate(
                      AllianceFlipUtil.mirror(
                          FieldConstants.getInstance()
                              .getTargetPoseForBarge(FieldGeometry.Barge.closeCage)
                              .plus(
                                  new Transform2d(new Translation2d(), Rotation2d.fromDegrees(15))),
                          isDownfield()));
              return pathToPoseRepulsorCommand(target, state -> target.getRotation());
            })
        .withName("Drive::PathToRightBarge");
  }

  public Command pathToCoralStationUndefendedCommand() {
    return defer(
            () -> {
              var targets =
                  List.of(
                      AllianceFlipUtil.rotate(
                          FieldConstants.getInstance().UndefendedIntakePoseLeft),
                      AllianceFlipUtil.rotate(
                          FieldConstants.getInstance().UndefendedIntakePoseRight));
              var target = getStateCopy().Pose.nearest(targets);
              return pathToPoseRepulsorCommand(target, state -> target.getRotation());
            })
        .withName("Drive::PathToCoralStationUndefended");
  }

  public Command pathToCoralStationDefendedForwardCommand() {
    return defer(
            () -> {
              var targets =
                  List.of(
                      AllianceFlipUtil.rotate(
                          FieldConstants.getInstance().DefendedIntakeForwardLeft),
                      AllianceFlipUtil.rotate(
                          FieldConstants.getInstance().DefendedIntakeForwardRight));
              var target = getStateCopy().Pose.nearest(targets);
              return pathToPoseRepulsorCommand(target, state -> target.getRotation());
            })
        .withName("Drive::PathToCoralStationDefendedForward");
  }

  public Command pathToCoralStationDefendedRearCommand() {
    return defer(
            () -> {
              var targets =
                  List.of(
                      AllianceFlipUtil.rotate(FieldConstants.getInstance().DefendedIntakeBackLeft),
                      AllianceFlipUtil.rotate(
                          FieldConstants.getInstance().DefendedIntakeBackRight));
              var target = getStateCopy().Pose.nearest(targets);
              return pathToPoseRepulsorCommand(target, state -> target.getRotation());
            })
        .withName("Drive::PathToCoralStationDefendedRear");
  }

  private Transform2d poseTolerance =
      new Transform2d(Inches.of(1), Inches.of(1), new Rotation2d(Degrees.of(5)));

  private Transform2d BargePoseTolerance =
      new Transform2d(Inches.of(6), Inches.of(95), new Rotation2d(Degrees.of(45)));

  public boolean IsAtPose(Pose2d target) {
    return IsAtPose(target, poseTolerance);
  }

  public boolean IsAtPoseBarge(Pose2d target) {
    return IsAtPose(target, BargePoseTolerance);
  }

  public boolean IsAtPose(Pose2d target, Transform2d tolerance) {

    var currentPose = getStateCopy().Pose;
    var error = target.minus(currentPose);

    RJLog.log("Drive/IsAtPose/X", error.getMeasureX().abs(Inches));
    RJLog.log("Drive/IsAtPose/Y", error.getMeasureY().abs(Inches));
    RJLog.log("Drive/IsAtPose/Theta", error.getRotation().getMeasure().abs(Degrees));

    var isAtPose =
        error.getTranslation().getNorm() < tolerance.getTranslation().getNorm()
            && abs(error.getRotation().getDegrees()) < tolerance.getRotation().getDegrees();

    RJLog.log("Drive/IsAtPose/Result", isAtPose);
    SmartDashboard.putBoolean("Drive/IsAtPose/Result", isAtPose);

    return isAtPose;
  }

  public boolean IsAlignedToBranch() {
    var state = getStateCopy();

    var target = state.Pose.nearest(FieldConstants.getInstance().AllRobotPoses);

    return IsAtPose(target);
  }

  public boolean IsAlignedBarge() {
    var target = AllianceFlipUtil.rotate(FieldGeometry.Barge.ScoreBargeCenter);

    return IsAtPoseBarge(target);
  }

  public boolean IsAlignedBargeMirrored() {
    var target = AllianceFlipUtil.rotate(FieldGeometry.Barge.ScoreBargeCenterMirrored);

    return IsAtPoseBarge(target);
  }

  RepulsorFieldPlanner repulsor = new RepulsorFieldPlanner();

  public Command pathToPoseRepulsorCommand(
      Pose2d target, Function<SwerveDriveState, Rotation2d> headingOverride) {

    var approachTolerance =
        new Transform2d(Meters.of(1.5), Meters.zero(), new Rotation2d(Degrees.of(180)));
    var convergeTolerance =
        new Transform2d(Inches.of(12), Inches.zero(), new Rotation2d(Degrees.of(180)));

    return sequence(
            run(() -> {
                  var state = getStateCopy();
                  var orientation = headingOverride.apply(state);

                  var input =
                      repulsor.sampleField(
                          state.Pose.getTranslation(),
                          target.getTranslation(),
                          orientation,
                          TunerConstants.kSpeedAt12Volts.in(MetersPerSecond),
                          2.5);

                  followPath(input);
                })
                .until(() -> IsAtPose(target, approachTolerance)),
            defer(
                    () -> {
                      if (IsAtPose(target, convergeTolerance)) {
                        return none();
                      }

                      var state = getStateCopy();
                      var fieldRelativeSpeeds =
                          state.Speeds.toFieldRelative(state.Pose.getRotation());

                      var linearSpeeds =
                          new Translation2d(fieldRelativeSpeeds.vx, fieldRelativeSpeeds.vy);

                      var waypoints =
                          PathPlannerPath.waypointsFromPoses(
                              new Pose2d(state.Pose.getTranslation(), linearSpeeds.getAngle()),
                              target);
                      var path =
                          new PathPlannerPath(
                              waypoints,
                              ppConstraints,
                              new IdealStartingState(
                                  linearSpeeds.getNorm(), getStateCopy().Pose.getRotation()),
                              new GoalEndState(0, target.getRotation()));
                      return new FollowPathCommand(
                          path,
                          () -> getStateCopy().Pose,
                          () -> getStateCopy().Speeds,
                          (speeds, feedforwards) ->
                              setControl(
                                  applyRobotSpeeds
                                      .withSpeeds(speeds)
                                      .withWheelForceFeedforwardsX(
                                          feedforwards.robotRelativeForcesXNewtons())
                                      .withWheelForceFeedforwardsY(
                                          feedforwards.robotRelativeForcesYNewtons())),
                          new PPHolonomicDriveController(translationConstants, rotationConstants),
                          ppRobotConfig,
                          () -> false,
                          this);
                    })
                .unless(() -> IsAtPose(target, convergeTolerance)),
            HoldState(target, new ChassisSpeeds()))
        .withName("Drive::PathToPoseRepulsor");
  }

  public Command pathToBranchRepulsor(Side side, int tag) {
    if (tag == 0) {
      return none();
    }
    var targetBranch = FieldConstants.getInstance().RobotPose(tag, side);

    var tolerance = new Transform2d(Meters.of(1), Meters.zero(), Rotation2d.k180deg);

    return pathToPoseRepulsorCommand(
            targetBranch,
            state -> {
              if (IsAtPose(targetBranch, tolerance)) {
                return targetBranch.getRotation();
              }

              var reefPose = AllianceFlipUtil.mirror(FieldGeometry.Reef.center, isDownfield());

              return AllianceFlipUtil.rotate(reefPose)
                  .minus(state.Pose.getTranslation())
                  .getAngle();
            })
        .withName(String.format("Drive::PathToBranch(%s)", side));
  }
}
