// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.oi.ControlMode.CLIMB_MODE;
import static frc.robot.oi.ControlMode.CYCLING_MODE;
import static frc.robot.oi.StructureState.*;

import choreo.auto.AutoChooser;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.auto.Autos;
import frc.robot.constants.Constants.AlgaeArmConfig;
import frc.robot.constants.Constants.AlgaeMechConfig;
import frc.robot.constants.Constants.ClimberConfig;
import frc.robot.constants.Constants.ControlsConfig;
import frc.robot.constants.Constants.CoralMechConfig;
import frc.robot.constants.Constants.ElevatorConfig;
import frc.robot.constants.Constants.FunnelConfig;
import frc.robot.constants.Constants.LEDConfig;
import frc.robot.constants.Constants.SuperstructureConfig;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.FieldConstants.Side;
import frc.robot.constants.TunerConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.lib.RJLog;
import frc.robot.oi.CommandGuliKitController;
import frc.robot.oi.ControlModeManager;
import frc.robot.oi.DriverControls;
import frc.robot.oi.OperatorControls;
import frc.robot.oi.StructureState;
import frc.robot.subsystems.AlgaeArm;
import frc.robot.subsystems.AlgaeMech;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CoralMech;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Funnel;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import java.util.Map;

public class RobotContainer {

  public final CommandGuliKitController driver = new CommandGuliKitController(0);
  public final CommandPS4Controller operator = new CommandPS4Controller(1);

  private final ControlsConfig controlsConfig = ControlsConfig.DEFAULT;

  public final DriverControls driverControls = new DriverControls(driver.getHID(), controlsConfig);

  private final OperatorControls operatorControls =
      new OperatorControls(operator.getHID(), controlsConfig);

  public final CommandGenericHID simHID = new CommandGenericHID(2);

  private final Telemetry logger =
      new Telemetry(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond));

  public final ControlModeManager controlModeManager = new ControlModeManager(controlsConfig);

  public final Elevator elevator = new Elevator(ElevatorConfig.DEFAULT);
  public final AlgaeArm algaeArm = new AlgaeArm(AlgaeArmConfig.DEFAULT);
  private final CoralMech coralMech = new CoralMech(CoralMechConfig.DEFAULT);
  private final AlgaeMech algaeMech = new AlgaeMech(AlgaeMechConfig.DEFAULT);
  private final Drive drivetrain = TunerConstants.createDrivetrain();
  private final Vision vision = getVisionSystem();
  private final Climber climber = new Climber(ClimberConfig.DEFAULT);
  public final Superstructure superstructure =
      new Superstructure(elevator, algaeArm, SuperstructureConfig.DEFAULT);
  private final LEDs leds = new LEDs(LEDConfig.DEFAULT);

  private final Funnel funnel = new Funnel(FunnelConfig.DEFAULT);

  private final Autos autos =
      new Autos(
          drivetrain, superstructure, coralMech, algaeMech, funnel, elevator, this::AutonAutoScore);
  private final AutoChooser autoChooser = new AutoChooser();
  private final SendableChooser<Command> ppAutoChooser = AutoBuilder.buildAutoChooser();

  private final SendableChooser<String> cageChooser = new SendableChooser<>();
  private final SendableChooser<String> bargeAlignmentChooser = new SendableChooser<>();

  private final Notifier alertsTask;
  private final Notifier brakeModeTask;
  private final Notifier coastModeTask;

  public RobotContainer() {
    configureBehavior();
    configureBindings();
    // sysidBindings();
    if (RobotBase.isSimulation()) {
      configureSimBindings();
    }

    drivetrain.registerTelemetry(logger::telemeterize);

    autoChooser.addCmd("PathPlanner Path", ppAutoChooser::getSelected);
    autos.configureAutoChooser(autoChooser);

    SmartDashboard.putData("Pathplanner Paths", ppAutoChooser);
    SmartDashboard.putData("Autons", autoChooser);
    SmartDashboard.putData("Cages", cageChooser);
    SmartDashboard.putData("Barge Scoring Position", bargeAlignmentChooser);

    leds.setDefaultCommand(leds.CyclingModeCommand(() -> defenseMode));

    alertsTask =
        new Notifier(
            () -> {
              RJLog.log("DefenseMode", defenseMode);
              elevator.refreshAlert();
              algaeArm.refreshAlert();
              coralMech.refreshAlert();
              algaeMech.refreshAlert();
              drivetrain.refreshAlert();
              climber.refreshAlert();
            });

    alertsTask.setName("Alerts");
    alertsTask.startPeriodic(0.25);

    brakeModeTask =
        new Notifier(
            () -> {
              drivetrain.configNeutralMode(NeutralModeValue.Brake);
              elevator.setBrakeMode();
            });

    coastModeTask =
        new Notifier(
            () -> {
              drivetrain.configNeutralMode(NeutralModeValue.Coast);
              elevator.setCoastMode();
            });
  }

  private void configureBehavior() {
    RobotModeTriggers.autonomous().and(() -> !elevator.zeroed()).onTrue(elevator.zeroCommand());
    RobotModeTriggers.teleop().and(() -> !elevator.zeroed()).onTrue(elevator.zeroCommand());

    // Zero forward because we should zero against something within the robot (forward is upward)
    RobotModeTriggers.autonomous()
        .and(() -> !algaeArm.zeroed())
        .onTrue(algaeArm.zeroForwardCommand());
    RobotModeTriggers.teleop().and(() -> !algaeArm.zeroed()).onTrue(algaeArm.zeroForwardCommand());

    RobotModeTriggers.autonomous()
        .and(algaeArm::zeroed)
        .and(elevator::zeroed)
        .onTrue(superstructure.Home());

    RobotModeTriggers.teleop()
        .and(algaeArm::zeroed)
        .and(elevator::zeroed)
        .onTrue(superstructure.HoldCurrentState());

    RobotModeTriggers.disabled()
        .whileTrue(
            Commands.waitTime(Milliseconds.of(5))
                .andThen(
                    runOnce(
                            () -> {
                              coastModeTask.startSingle(0);
                            })
                        .withName("SetElevatorAndDriveCoastMode"))
                .andThen(elevator.goToPositionCommand(elevator::getPosition))
                .ignoringDisable(true)
                .withName("CoastMechanismsWhenDisabled"))
        .negate()
        .onTrue(
            runOnce(
                    () -> {
                      brakeModeTask.startSingle(0);
                    })
                .withName("BrakeMechanismsWhenEnabled"));

    new Trigger(() -> funnel.DetectCoral())
        .and(
            () ->
                superstructure.getCurrentConfiguration().isHomeConfig()
                    && elevator.isReady(SuperstructureConfig.DEFAULT.SetpointElevatorTolerance()))
        .and(() -> !coralMech.hasCoral())
        .onTrue(LoadCoral().withTimeout(Seconds.of(5)).withName("LoadCoral"));

    new Trigger(() -> coralMech.coralDetected())
        .onTrue(leds.CoralDetected().withTimeout(Seconds.of(2)))
        .onFalse(leds.CoralScored().withTimeout(Seconds.of(2)));

    new Trigger(() -> algaeMech.HoldsAlgae())
        .onTrue(leds.AlgaeDetected().withTimeout(Seconds.of(2)))
        .onFalse(leds.AlgaeScored().withTimeout(Seconds.of(2)));

    new Trigger(() -> superstructure.getTargetScoringConfiguration().isScoreAlgaeConfig())
        .and(new Trigger(drivetrain::IsAlignedBarge).or(drivetrain::IsAlignedBargeMirrored))
        .whileTrue(leds.BargeSafe());

    new Trigger(() -> superstructure.getTargetScoringConfiguration().isScoreAlgaeConfig())
        .and(() -> !drivetrain.IsAlignedBarge())
        .and(() -> !drivetrain.IsAlignedBargeMirrored())
        .whileTrue(leds.BargeUnsafe());

    RobotModeTriggers.disabled().whileTrue(drivetrain.stopCommand());

    RobotModeTriggers.autonomous()
        .whileTrue(autoChooser.selectedCommandScheduler().withName("Auton::SelectedCommand"));
  }

  private void configureBindings() {
    // Driver - Drive Controls
    RobotModeTriggers.teleop()
        .onTrue(
            runOnce(
                () ->
                    drivetrain.setDefaultCommand(
                        drivetrain.teleopCommand(
                            driverControls,
                            controlModeManager::getCurrentMode,
                            superstructure::getTargetScoringConfiguration,
                            vision::enabled))))
        .onFalse(runOnce(() -> drivetrain.setDefaultCommand(drivetrain.stopCommand())));

    // Driver - Reset Forward
    driver
        .start()
        .onTrue(
            drivetrain
                .runOnce(() -> drivetrain.seedFieldCentric())
                .withName("Drive::SeedFieldCentric"));
    // Driver - Disable cameras
    driver.back().toggleOnTrue(vision.disableCommand());

    // Operator - Enter Cycling Mode
    operator.touchpad().onTrue(controlModeManager.goToMode(CYCLING_MODE));

    // Operator - Enter Climb Mode
    operator.PS().onTrue(controlModeManager.goToMode(CLIMB_MODE));

    // Operator - Zero Elevator
    operator.share().onTrue(elevator.zeroCommand());
    // Operator - Zero Arm
    operator.options().onTrue(algaeArm.zeroForwardCommand());

    // Operator Manual Controls - Elevator
    new Trigger(() -> operatorControls.manualElevator() != 0)
        .whileTrue(elevator.manualCommand(operatorControls::manualElevator))
        .onFalse(elevator.goToCurrentPositionCommand());

    // Operator Manual Controls - Arm
    new Trigger(() -> operatorControls.manualArm() != 0)
        .whileTrue(algaeArm.manualCommand(operatorControls::manualArm))
        .onFalse(algaeArm.goToCurrentPositionCommand());

    configureCyclingBindings();
    configureClimbingBindings();
  }

  private void configureCyclingBindings() {
    var cyclingMode = controlModeManager.isMode(CYCLING_MODE);

    // Queued-Mode Operator Controls
    // Score Coral L4
    cyclingMode
        .and(operator.triangle())
        .onTrue(
            either(
                    superstructure.setTargetScoringConfigurationCommand(SCORE_ALGAE_BARGE),
                    superstructure.setTargetScoringConfigurationCommand(SCORE_CORAL_L4),
                    operator.getHID()::getL1Button)
                .ignoringDisable(true)
                .withName("Superstructure::SetHeightL4"));

    // Score at L3
    cyclingMode
        .and(operator.square())
        .onTrue(
            either(
                    superstructure.setTargetScoringConfigurationCommand(INTAKE_ALGAE_L3),
                    superstructure.setTargetScoringConfigurationCommand(SCORE_CORAL_L3),
                    operator.getHID()::getL1Button)
                .ignoringDisable(true)
                .withName("Superstructure::SetHeightL3"));

    // Score at L2
    cyclingMode
        .and(operator.cross())
        .onTrue(
            either(
                    superstructure.setTargetScoringConfigurationCommand(INTAKE_ALGAE_L2),
                    superstructure.setTargetScoringConfigurationCommand(SCORE_CORAL_L2),
                    operator.getHID()::getL1Button)
                .ignoringDisable(true)
                .withName("Superstructure::SetHeightL2"));

    // Score at L1
    cyclingMode
        .and(operator.circle())
        .onTrue(
            either(
                    superstructure.setTargetScoringConfigurationCommand(SCORE_ALGAE_PROCESSOR),
                    superstructure.setTargetScoringConfigurationCommand(SCORE_CORAL_L1),
                    operator.getHID()::getL1Button)
                .ignoringDisable(true)
                .withName("Superstructure::SetHeightL1"));

    // Configure Superstructure to target
    cyclingMode
        .and(driver.leftTrigger())
        .whileTrue(superstructure.GoToTarget())
        .and(
            () ->
                elevator.isReady(
                    StructureState.HOME.ElevatorHeight(),
                    Inches.of(5),
                    InchesPerSecond.of(0),
                    InchesPerSecond.of(100)))
        .onTrue(
            runOnce(
                () -> {
                  var intakeCommand = coralMech.getCurrentCommand();
                  if (intakeCommand != null) {
                    CommandScheduler.getInstance().cancel(intakeCommand);
                  }
                }));

    // Home Elevator
    cyclingMode
        // Driver not holding aim, driver and operator not intaking algae
        .and(driver.leftTrigger().or(driver.rightBumper()).or(operator.R1()).negate())
        .onTrue(superstructure.Home());

    pathToNearestBranchBindings();
    pathToAnyBranchBindings();

    // Intake with coral mech
    cyclingMode
        .and(driver.rightLowerPaddle().or(driver.leftLowerPaddle()).or(operator.povLeft()))
        .and(() -> superstructure.WithinTolerance(HOME))
        .whileTrue(LoadCoral().alongWith(leds.CoralIntaking()));

    // Configure Superstructure to Intake from Floor
    cyclingMode
        .and(driver.rightBumper().or(operator.povRight()))
        .and(this::isNotScoring)
        .whileTrue(LoadAlgae().alongWith(leds.AlgaeIntaking()))
        .onFalse(
            either(superstructure.GoToTarget(), superstructure.Home(), driver.leftTrigger())
                .withName("Superstructure::ReturnToStateAfterAlgaePickup"));

    // Bring Algae Arm back into Frame after algae is picked up
    cyclingMode
        .and(algaeMech::HoldsAlgae)
        .onTrue(algaeArm.HomeCommand().alongWith(leds.SetColorCommand(Color.kTeal).asProxy()));

    // Auto Score
    cyclingMode
        .and(driver.rightTrigger())
        .and(() -> superstructure.getTargetScoringConfiguration().isScoringConfig())
        .whileTrue(ConditionalAutoScore());

    // Operator Manual Controls - Algae mech and Coral mech
    cyclingMode
        .and(() -> operatorControls.climbertrigger() != 0)
        .whileTrue(
            either(
                    algaeMech.RunAlgaePower(operatorControls::climbertrigger),
                    coralMech
                        .RunCoralPower(operatorControls::climbertrigger)
                        .alongWith(funnel.FunnelPower(operatorControls::climbertrigger)),
                    operator.getHID()::getL1Button)
                .withName("Operator::FiddleWithEndEffector"));

    bargeAlignmentChooser.setDefaultOption("Left Cage", "Left Cage");
    bargeAlignmentChooser.addOption("Right Cage", "Right Cage");

    cyclingMode
        .and(driver.leftBumper())
        .and(driver.leftTrigger().negate())
        .and(vision::enabled)
        .whileTrue(
            select(
                    Map.of(
                        "Left Cage",
                        drivetrain.pathToLeftBargeCommand(),
                        "Right Cage",
                        drivetrain.pathToRightBargeCommand()),
                    bargeAlignmentChooser::getSelected)
                .withName("Drive::PathToAutoBarge"));

    cyclingMode
        .and(driver.leftBumper().negate())
        .onTrue(
            runOnce(
                    () -> {
                      var driveCommand = drivetrain.getCurrentCommand();
                      if (driveCommand != null
                          && driveCommand.getName().startsWith("Drive::PathToAutoBarge")) {
                        CommandScheduler.getInstance().cancel(driveCommand);
                      }
                    })
                .withName("Drive::CancelBargeAutoAlign"));

    cyclingMode.and(operator.povDown()).onTrue(runOnce(() -> defenseMode = true));
    cyclingMode.and(operator.povUp()).onTrue(runOnce(() -> defenseMode = false));

    cyclingMode
        .and(driver.rightLowerPaddle())
        .and(vision::enabled)
        .whileTrue(
            either(
                drivetrain.pathToCoralStationDefendedForwardCommand(),
                drivetrain.pathToCoralStationUndefendedCommand(),
                () -> defenseMode));
    cyclingMode
        .and(driver.leftLowerPaddle())
        .and(vision::enabled)
        .whileTrue(
            either(
                drivetrain.pathToCoralStationDefendedRearCommand(),
                drivetrain.pathToCoralStationUndefendedCommand(),
                () -> defenseMode));
  }

  private void pathToNearestBranchBindings() {
    var cyclingMode = controlModeManager.isMode(CYCLING_MODE);

    // Align to left branch
    cyclingMode
        .and(() -> !controlsConfig.PathToAnyFace())
        .and(vision::enabled)
        .and(driver.leftUpperPaddle())
        .whileTrue(
            drivetrain
                .pathToBranchCommand(controlsConfig, Side.LEFT)
                .alongWith(vision.DisableRearCamerasCommand())
                .until(coralMech::lostCoral)
                .withName("Drivetrain::PathToBranch(Left)"));

    // Align to right branch
    cyclingMode
        .and(() -> !controlsConfig.PathToAnyFace())
        .and(vision::enabled)
        .and(driver.rightUpperPaddle())
        .whileTrue(
            drivetrain
                .pathToBranchCommand(controlsConfig, Side.RIGHT)
                .alongWith(vision.DisableRearCamerasCommand())
                .until(coralMech::lostCoral)
                .withName("Drivetrain::PathToBranch(Right)"));
  }

  // This is super verbose, but it enables the driver to switch the reef face they point to at-will
  // and the command will switch accordingly.
  private void pathToAnyBranchBindings() {
    var cyclingMode =
        controlModeManager
            .isMode(CYCLING_MODE)
            .and(controlsConfig::PathToAnyFace)
            .and(vision::enabled);

    cyclingMode
        .and(driver.leftUpperPaddle().or(driver.rightUpperPaddle()).negate())
        .onTrue(
            runOnce(
                    () -> {
                      var driveCommand = drivetrain.getCurrentCommand();
                      if (driveCommand != null
                          && driveCommand.getName().startsWith("Drivetrain::PathToBranch")) {
                        CommandScheduler.getInstance().cancel(driveCommand);
                      }
                    })
                .withName("Drive::CancelAutoAlign"));

    // Align to close face, left branch
    cyclingMode
        .and(driver.leftTrigger().negate())
        .and(driver.leftUpperPaddle())
        .and(() -> driverControls.rightStickBucketedAngle() == -90)
        .onTrue(AlignToReef(Side.LEFT, 18, 10, 7, 21, "A"));

    // Align to close face, right branch
    cyclingMode
        .and(driver.leftTrigger().negate())
        .and(driver.rightUpperPaddle())
        .and(() -> driverControls.rightStickBucketedAngle() == -90)
        .onTrue(AlignToReef(Side.RIGHT, 18, 10, 7, 21, "B"));

    // Align to close right face, left branch
    cyclingMode
        .and(driver.leftTrigger().negate())
        .and(driver.leftUpperPaddle())
        .and(() -> driverControls.rightStickBucketedAngle() == -30)
        .onTrue(AlignToReef(Side.LEFT, 17, 11, 8, 20, "C"));

    // Align to close right face, right branch
    cyclingMode
        .and(driver.leftTrigger().negate())
        .and(driver.rightUpperPaddle())
        .and(() -> driverControls.rightStickBucketedAngle() == -30)
        .onTrue(AlignToReef(Side.RIGHT, 17, 11, 8, 20, "D"));

    // Align to far right face, left branch
    cyclingMode
        .and(driver.leftTrigger().negate())
        .and(driver.leftUpperPaddle())
        .and(() -> driverControls.rightStickBucketedAngle() == 30)
        .onTrue(AlignToReef(Side.LEFT, 22, 6, 9, 19, "E"));

    // Align to far right face, right branch
    cyclingMode
        .and(driver.leftTrigger().negate())
        .and(driver.rightUpperPaddle())
        .and(() -> driverControls.rightStickBucketedAngle() == 30)
        .onTrue(AlignToReef(Side.RIGHT, 22, 6, 9, 19, "F"));

    // Align to far face, left branch
    cyclingMode
        .and(driver.leftTrigger().negate())
        .and(driver.leftUpperPaddle())
        .and(() -> driverControls.rightStickBucketedAngle() == 90)
        .onTrue(AlignToReef(Side.LEFT, 21, 7, 10, 18, "G"));

    // Align to far face, right branch
    cyclingMode
        .and(driver.leftTrigger().negate())
        .and(driver.rightUpperPaddle())
        .and(() -> driverControls.rightStickBucketedAngle() == 90)
        .onTrue(AlignToReef(Side.RIGHT, 21, 7, 10, 18, "H"));

    // Align to far left face, left branch
    cyclingMode
        .and(driver.leftTrigger().negate())
        .and(driver.leftUpperPaddle())
        .and(() -> driverControls.rightStickBucketedAngle() == 150)
        .onTrue(AlignToReef(Side.LEFT, 20, 8, 11, 17, "I"));

    // Align to far left face, right branch
    cyclingMode
        .and(driver.leftTrigger().negate())
        .and(driver.rightUpperPaddle())
        .and(() -> driverControls.rightStickBucketedAngle() == 150)
        .onTrue(AlignToReef(Side.RIGHT, 20, 8, 11, 17, "J"));

    // Align to close left face, left branch
    cyclingMode
        .and(driver.leftTrigger().negate())
        .and(driver.leftUpperPaddle())
        .and(() -> driverControls.rightStickBucketedAngle() == -150)
        .onTrue(AlignToReef(Side.LEFT, 19, 9, 6, 22, "K"));

    // Align to close left face, right branch
    cyclingMode
        .and(driver.leftTrigger().negate())
        .and(driver.rightUpperPaddle())
        .and(() -> driverControls.rightStickBucketedAngle() == -150)
        .onTrue(AlignToReef(Side.RIGHT, 19, 9, 6, 22, "L"));
  }

  private Command waitForReconfigure() {
    return waitUntil(superstructure::WithinTolerance);
  }

  private Command waitForAlignment() {
    return waitUntil(drivetrain::IsAlignedToBranch);
  }

  private int AlgaeLevelOnClosestReefFace() {
    if (!vision.enabled()) {
      return 0;
    }

    var state = drivetrain.getStateCopy();
    var algaeLevel = FieldConstants.getInstance().AlgaeHeightOnClosestReefSide(state.Pose);
    return algaeLevel;
  }

  private void configureClimbingBindings() {
    var climbMode = controlModeManager.isMode(CLIMB_MODE);
    climbMode.whileTrue(leds.ClimberModeCommand());

    // Operator Manual Controls - Climber
    climbMode
        .and(() -> operatorControls.climbertrigger() != 0)
        .whileTrue(climber.manualCommand(operatorControls::climbertrigger))
        .onFalse(climber.idleCommand());

    // Superstructure to Climb mode
    climbMode.and(driver.leftTrigger()).whileTrue(superstructure.Climb());

    climbMode
        .and(operator.L1())
        .onTrue(runOnce(climber::resetPosition))
        .whileTrue(climber.ClimberDeploy());

    climbMode.and(operator.square()).whileTrue(funnel.FunnelSolenoidDeploy());
    // Path to Cage
    climbMode.and(driver.y()).and(vision::enabled).whileTrue(drivetrain.pathToLeftCageCommand());
    climbMode.and(driver.x()).and(vision::enabled).whileTrue(drivetrain.pathToMiddleCageCommand());
    climbMode.and(driver.a()).and(vision::enabled).whileTrue(drivetrain.pathToRightCageCommand());

    // Path to Cage - Selectable
    cageChooser.setDefaultOption("Left Cage", "Left Cage");
    cageChooser.addOption("Middle Cage", "Middle Cage");
    cageChooser.addOption("Right Cage", "Right Cage");

    climbMode
        .and(driver.b())
        .and(vision::enabled)
        .whileTrue(
            select(
                    Map.of(
                        "Left Cage",
                        drivetrain.pathToLeftCageCommand(),
                        "Middle Cage",
                        drivetrain.pathToMiddleCageCommand(),
                        "Right Cage",
                        drivetrain.pathToRightCageCommand()),
                    cageChooser::getSelected)
                .withName("Drive::PathToAutoCage"));
  }

  private void sysidBindings() {
    drivetrain.setDefaultCommand(
        drivetrain.teleopCommand(
            driverControls,
            controlModeManager::getCurrentMode,
            superstructure::getTargetScoringConfiguration,
            vision::enabled));

    // driver.x().whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    // driver.y().whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
    // driver.a().whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    // driver.b().whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    driver.a().whileTrue(elevator.sysIdRoutine());

    // new Trigger(() -> operatorControls.manualElevator() != 0)
    //     .whileTrue(elevator.manualCommand(operatorControls::manualElevator))
    //     .onFalse(elevator.manualCommand(() -> 0.0));
  }

  private void configureSimBindings() {
    SwerveRequest.FieldCentric fieldCentric =
        new SwerveRequest.FieldCentric()
            .withDriveRequestType(
                DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    drivetrain.setDefaultCommand(
        // Drivetrain will execute this command periodically
        drivetrain
            .applyRequest(
                () ->
                    fieldCentric
                        .withVelocityX(
                            ControlsConfig.DEFAULT.MaxSpeed().times(-simHID.getRawAxis(1)))
                        .withVelocityY(
                            ControlsConfig.DEFAULT.MaxSpeed().times(-simHID.getRawAxis(0)))
                        .withRotationalRate(
                            ControlsConfig.DEFAULT.MaxAngularRate().times(-simHID.getRawAxis(2))))
            .withName("Drive::Teleop"));

    // simHID.button(1).whileTrue(drivetrain.autoAlignToBranchCommand(Side.LEFT));
    // simHID.button(2).whileTrue(drivetrain.autoAlignToBranchCommand(Side.RIGHT));
    // simHID.button(3).whileTrue(drivetrain.pathToBranchCommand(Side.LEFT));
    // simHID.button(4).whileTrue(drivetrain.pathToBranchCommand(Side.RIGHT));

    // simHID.button(1).whileTrue(drivetrain.pathToLeftCageCommand());
    // simHID.button(2).whileTrue(drivetrain.pathToMiddleCageCommand());
    // simHID.button(3).whileTrue(drivetrain.pathToRightCageCommand());

    // simHID.button(4).whileTrue(cageChooser.getSelected());

    // Align to left branch pointed to by stick

    simHID
        .button(1)
        .whileTrue(
            drivetrain
                .pathToBranchRepulsor(Side.LEFT, 17)
                .alongWith(vision.DisableRearCamerasCommand())
                .withName("Drive::PathToReefFace(LEFT)"));
    simHID
        .button(2)
        .whileTrue(
            drivetrain
                .pathToBranchRepulsor(Side.LEFT, 18)
                .alongWith(vision.DisableRearCamerasCommand())
                .withName("Drive::PathToReefFace(LEFT)"));
    simHID
        .button(3)
        .whileTrue(
            drivetrain
                .pathToBranchRepulsor(Side.LEFT, 19)
                .alongWith(vision.DisableRearCamerasCommand())
                .withName("Drive::PathToReefFace(LEFT)"));
    simHID.button(4).whileTrue(drivetrain.pathToCoralStationUndefendedCommand());

    simHID.button(5).whileTrue(drivetrain.pathToCoralStationDefendedRearCommand());
    simHID.button(6).whileTrue(drivetrain.pathToCoralStationDefendedForwardCommand());
  }

  public Vision getVisionSystem() {
    if (RobotBase.isReal()) {
      return new Vision(
          drivetrain::addVisionMeasurement,
          new VisionIOPhotonVision(VisionConstants.camera0Name, VisionConstants.robotToCamera0),
          new VisionIOPhotonVision(VisionConstants.camera1Name, VisionConstants.robotToCamera1));
      // ,
      // new VisionIOPhotonVision(VisionConstants.camera2Name, VisionConstants.robotToCamera2),
      // new VisionIOPhotonVision(VisionConstants.camera3Name, VisionConstants.robotToCamera3));
    }

    return new Vision(
        drivetrain::addVisionMeasurement,
        new VisionIOPhotonVisionSim(
            VisionConstants.camera0Name,
            VisionConstants.robotToCamera0,
            () -> drivetrain.getStateCopy().Pose,
            VisionConstants.camera0SimProperties.get()),
        new VisionIOPhotonVisionSim(
            VisionConstants.camera1Name,
            VisionConstants.robotToCamera1,
            () -> drivetrain.getStateCopy().Pose,
            VisionConstants.camera0SimProperties.get()),
        new VisionIOPhotonVisionSim(
            VisionConstants.camera2Name,
            VisionConstants.robotToCamera2,
            () -> drivetrain.getStateCopy().Pose,
            VisionConstants.camera0SimProperties.get()),
        new VisionIOPhotonVisionSim(
            VisionConstants.camera3Name,
            VisionConstants.robotToCamera3,
            () -> drivetrain.getStateCopy().Pose,
            VisionConstants.camera0SimProperties.get()));
  }

  public Command LoadCoral() {
    return coralMech.Intake().deadlineFor(funnel.Intake()).withName("LoadCoral");
  }

  public Command LoadAlgae() {
    return algaeMech
        .Intake()
        .deadlineFor(
            // If in a reef config, intake from algae level on closest face
            either(
                select(
                    Map.of(
                        0,
                        none(),
                        2,
                        superstructure.IntakeAlgaeL2().asProxy(),
                        3,
                        superstructure.IntakeAlgaeL3().asProxy()),
                    this::AlgaeLevelOnClosestReefFace),
                // If not in a reef config, intake from floor
                superstructure.IntakeAlgaeFloor().asProxy(),
                () -> superstructure.getCurrentConfiguration().isReefConfig()))
        .withName("LoadAlgae");
  }

  private Command ConditionalAutoScore() {
    return waitForReconfigure()
        // Wait for Alignment only if driver is auto-aiming
        .alongWith(
            waitForAlignment()
                .onlyIf(
                    driver.leftUpperPaddle().or(driver.rightUpperPaddle()).and(vision::enabled)),
            runOnce(this::markScoring))
        .deadlineFor(leds.BlinkColorCommand(Color.kLimeGreen).asProxy())
        .andThen(AutoScore().alongWith(leds.SetColorCommand(Color.kRed).asProxy()))
        .finallyDo(this::markNotScoring)
        .withName("ConditionalAutoScore");
  }

  public Command AutonAutoScore(StructureState scoreState) {
    var bumpUp =
        scoreState == SCORE_CORAL_L1
            ? SuperstructureConfig.DEFAULT.AutoScoreBumpUpHeightL1()
            : SuperstructureConfig.DEFAULT.AutoScoreBumpUpHeightL4();
    return coralMech
        .Score(() -> scoreState)
        .deadlineFor(
            Commands.wait(SuperstructureConfig.DEFAULT.AutoScoreBumpUpWaitTime().in(Seconds))
                .andThen(
                    elevator
                        .defer(
                            () -> elevator.goToPositionCommand(elevator.getPosition().plus(bumpUp)))
                        .onlyIf(scoreState::requiresBumpUp)))
        .withName("AutoScore");
  }

  public Command AutoScore() {
    return either(
            algaeMech.Score(),
            // Don't try and score coral in configs at the hard stop
            either(
                coralMech
                    .Score(superstructure::getCurrentConfiguration)
                    .deadlineFor(
                        Commands.wait(
                                SuperstructureConfig.DEFAULT.AutoScoreBumpUpWaitTime().in(Seconds))
                            .andThen(
                                deferredProxy(
                                        () -> {
                                          var bumpUp =
                                              superstructure.getTargetScoringConfiguration()
                                                      == SCORE_CORAL_L1
                                                  ? SuperstructureConfig.DEFAULT
                                                      .AutoScoreBumpUpHeightL1()
                                                  : SuperstructureConfig.DEFAULT
                                                      .AutoScoreBumpUpHeightL4();
                                          return elevator.goToPositionCommand(
                                              elevator.getPosition().plus(bumpUp));
                                        })
                                    .onlyIf(
                                        () ->
                                            superstructure
                                                .getTargetScoringConfiguration()
                                                .requiresBumpUp()))),
                none(),
                () -> superstructure.getTargetScoringConfiguration().isScoreCoralConfig()),
            () -> superstructure.getTargetScoringConfiguration().isScoreAlgaeConfig())
        .withName("AutoScore");
  }

  private boolean scoring;
  private boolean defenseMode;

  private void markScoring() {
    scoring = true;
  }

  private void markNotScoring() {
    scoring = false;
  }

  private boolean isScoring() {
    return scoring;
  }

  private boolean isNotScoring() {
    return !scoring;
  }

  private Command AlignToReef(
      Side side, int blueOwnReef, int blueOppReef, int redOwnReef, int redOppReef, String branch) {
    return either(
            either(
                drivetrain.pathToBranchCommand(controlsConfig, Side.LEFT, redOppReef),
                drivetrain.pathToBranchCommand(controlsConfig, side, redOwnReef),
                drivetrain::isDownfield),
            either(
                drivetrain.pathToBranchCommand(controlsConfig, Side.LEFT, blueOppReef),
                drivetrain.pathToBranchCommand(controlsConfig, side, blueOwnReef),
                drivetrain::isDownfield),
            () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red)
        .alongWith(vision.DisableRearCamerasCommand())
        .until(coralMech::lostCoral)
        .withName(String.format("Drivetrain::PathToBranch(%s)", branch));
  }
}
