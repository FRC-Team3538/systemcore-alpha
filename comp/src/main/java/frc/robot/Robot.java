// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.util.PathPlannerLogging;
import dev.doglog.DogLogOptions;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.FieldGeometry;
import frc.robot.lib.CANSignalManager;
import frc.robot.lib.RJLog;
import frc.robot.lib.pathplanning.RepulsorFieldPlanner;

public class Robot extends TimedRobot {
  private final RobotContainer m_robotContainer;

  public Robot() {
    // RobotController.setBrownoutVoltage(Volts.of(6));
    RJLog.setOptions(new DogLogOptions().withCaptureNt(true).withCaptureDs(true));
    RJLog.setEnabled(true);
    m_robotContainer = new RobotContainer();
    // LoggedCommandScheduler.init(CommandScheduler.getInstance());

    SignalLogger.start();

    DriverStation.silenceJoystickConnectionWarning(true);

    // Log active commands
    CommandScheduler.getInstance().onCommandInitialize(RJLog::logInitializedCommand);
    CommandScheduler.getInstance().onCommandFinish(RJLog::logFinishedCommand);
    CommandScheduler.getInstance().onCommandInterrupt(RJLog::logInterruptedCommand);

    PathPlannerLogging.setLogActivePathCallback(path -> RJLog.log("PathPlanner/Path", path));
    PathPlannerLogging.setLogCurrentPoseCallback(
        pose -> RJLog.log("PathPlanner/CurrentPose", pose));
    PathPlannerLogging.setLogTargetPoseCallback(pose -> RJLog.log("PathPlanner/TargetPose", pose));

    CommandScheduler.getInstance()
        .schedule(
            FollowPathCommand.warmupCommand().withName("PathPlanner::FollowPathCommand[Warmup]"));

    RepulsorFieldPlanner planner = new RepulsorFieldPlanner();
    planner.sampleField(
        new Translation2d(FieldGeometry.fieldLength / 2, FieldGeometry.fieldWidth / 2),
        new Translation2d(FieldGeometry.fieldLength, FieldGeometry.fieldWidth),
        Rotation2d.k180deg,
        1,
        1);

    if (isReal()) {
      // NetworkTableInstance.getDefault()
      //     .startEntryDataLog(DataLogManager.getLog(), "/Tracer", "NT:/Tracer");
    }
  }

  @Override
  public void robotPeriodic() {
    Threads.setCurrentThreadPriority(true, 1);

    CANSignalManager.refreshSignals();
    CommandScheduler.getInstance().run();
    // Tracer.traceFunc("LoggedCommandScheduler::periodic", LoggedCommandScheduler::periodic);

    // RJLog.log(
    //     "DriverInput", m_robotContainer.driverControls.fieldCentric(ControlMode.CYCLING_MODE));

    RJLog.log("Driver/POV", m_robotContainer.driver.getHID().getPOV());

    RJLog.log("ControlMode", m_robotContainer.controlModeManager.getCurrentMode());
    RJLog.log(
        "SuperStructure/TargetState",
        m_robotContainer.superstructure.getTargetScoringConfiguration());
    RJLog.log(
        "SuperStructure/CurrentState", m_robotContainer.superstructure.getCurrentConfiguration());

    SmartDashboard.putString(
        "ControlMode", m_robotContainer.controlModeManager.getCurrentMode().toString());
    SmartDashboard.putString(
        "SuperStructure/TargetState",
        m_robotContainer.superstructure.getTargetScoringConfiguration().toString());
    SmartDashboard.putString(
        "SuperStructure/CurrentState",
        m_robotContainer.superstructure.getCurrentConfiguration().toString());

    var controller = m_robotContainer.driver.getHID();

    // RJLog.log("Driver/leftx", controller.getLeftX());
    // RJLog.log("Driver/lefty", controller.getLeftY());
    // RJLog.log("Driver/rightx", controller.getRightX());
    // RJLog.log("Driver/righty", controller.getRightY());
    // RJLog.log("Driver/lefttrigger", controller.getLeftTriggerAxis());
    // RJLog.log("Driver/righttrigger", controller.getRightTriggerAxis());

    // RJLog.log("Driver/a", controller.getAButton());
    // RJLog.log("Driver/b", controller.getBButton());
    // RJLog.log("Driver/x", controller.getXButton());
    // RJLog.log("Driver/y", controller.getYButton());
    // RJLog.log("Driver/leftbumper", controller.getLeftBumperButton());
    // RJLog.log("Driver/rightbumper", controller.getRightBumperButton());
    // RJLog.log("Driver/start", controller.getStartButton());
    // RJLog.log("Driver/back", controller.getBackButton());
    // RJLog.log("Driver/leftstick", controller.getLeftStickButton());
    // RJLog.log("Driver/rightstick", controller.getRightStickButton());
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
