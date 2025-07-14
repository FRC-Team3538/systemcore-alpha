package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.oi.StructureState.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants.SuperstructureConfig;
import frc.robot.lib.RJLog;
import frc.robot.oi.StructureState;
import java.util.HashMap;

public class Superstructure {
  private final Elevator elevator;
  private final AlgaeArm arm;

  private final SuperstructureConfig config;

  private StructureState targetScoringConfiguration;
  private StructureState currentConfiguration;

  public Superstructure(Elevator elevator, AlgaeArm arm, SuperstructureConfig config) {
    this.elevator = elevator;
    this.arm = arm;
    this.config = config;

    targetScoringConfiguration = config.DefaultScoreState();
    currentConfiguration = HOME;
  }

  public StructureState getTargetScoringConfiguration() {
    return targetScoringConfiguration;
  }

  public StructureState getCurrentConfiguration() {
    return currentConfiguration;
  }

  public void setTargetScoringConfiguration(StructureState target) {
    targetScoringConfiguration = target;
  }

  public Command setTargetScoringConfigurationCommand(StructureState target) {
    return runOnce(() -> setTargetScoringConfiguration(target))
        .withName(String.format("Superstructure::SetTargetScoringConfiguration(%s)", target));
  }

  public Command SetpointCommand(StructureState target) {
    return sequence(
            arm.goToPositionCommand(HOME::ArmPitch)
                .until(() -> arm.getPosition().gt(Degrees.of(100)))
                .unless(
                    () ->
                        currentConfiguration == target
                            && elevator.isReady(target.ElevatorHeight())),
            parallel(
                elevator.SetpointCommand(target),
                waitUntil(
                        () ->
                            elevator.isReady(
                                target.ElevatorHeight(),
                                config.IntermediaryElevatorTolerance(),
                                MetersPerSecond.zero(),
                                MetersPerSecond.of(1)))
                    .traced("WaitForElevatorAtPosition")
                    .andThen(arm.SetpointCommand(target).traced()),
                runOnce(() -> currentConfiguration = target)))
        .withName(String.format("Superstructure::%s", target));
  }

  public Command GoToTarget() {
    var commandMap = new HashMap<StructureState, Command>();
    commandMap.put(HOME, Home());
    commandMap.put(INTAKE_CORAL, IntakeCoral());
    commandMap.put(SCORE_CORAL_L1, ScoreCoralL1());
    commandMap.put(SCORE_CORAL_L2, ScoreCoralL2());
    commandMap.put(SCORE_CORAL_L3, ScoreCoralL3());
    commandMap.put(SCORE_CORAL_L4, ScoreCoralL4());
    commandMap.put(INTAKE_ALGAE_FLOOR, IntakeAlgaeFloor());
    commandMap.put(SCORE_ALGAE_PROCESSOR, ScoreAlgaeProcessor());
    commandMap.put(INTAKE_ALGAE_L2, IntakeAlgaeL2());
    commandMap.put(INTAKE_ALGAE_L3, IntakeAlgaeL3());
    commandMap.put(SCORE_ALGAE_BARGE, ScoreAlgaeBarge());
    commandMap.put(CLIMB, Climb());
    return select(commandMap, this::getTargetScoringConfiguration)
        .withName("Superstructure::GoToTarget");
  }

  public Command Home() {
    return SetpointCommand(HOME);
  }

  public Command IntakeCoral() {
    return SetpointCommand(INTAKE_CORAL);
  }

  public Command ScoreCoralL1() {
    return SetpointCommand(SCORE_CORAL_L1);
  }

  public Command ScoreCoralL2() {
    return SetpointCommand(SCORE_CORAL_L2);
  }

  public Command ScoreCoralL3() {
    return SetpointCommand(SCORE_CORAL_L3);
  }

  public Command ScoreCoralL4() {
    return SetpointCommand(SCORE_CORAL_L4);
  }

  public Command IntakeAlgaeFloor() {
    return SetpointCommand(INTAKE_ALGAE_FLOOR);
  }

  public Command ScoreAlgaeProcessor() {
    return SetpointCommand(SCORE_ALGAE_PROCESSOR);
  }

  public Command PrepareAlgaeL2() {
    return SetpointCommand(PRIME_INTAKE_ALGAE_L2);
  }

  public Command PrepareAlgaeL3() {
    return SetpointCommand(PRIME_INTAKE_ALGAE_L3);
  }

  public Command IntakeAlgaeL2() {
    return SetpointCommand(INTAKE_ALGAE_L2);
  }

  public Command IntakeAlgaeL3() {
    return SetpointCommand(INTAKE_ALGAE_L3);
  }

  public Command ScoreAlgaeBarge() {
    return SetpointCommand(SCORE_ALGAE_BARGE);
  }

  public Command Climb() {
    return SetpointCommand(CLIMB);
  }

  public Command HoldCurrentState() {
    return elevator
        .goToCurrentPositionCommand()
        .alongWith(arm.goToCurrentPositionCommand())
        .withName("Superstructure::HoldCurrentPosition");
  }

  public boolean WithinTolerance() {
    return WithinTolerance(currentConfiguration);
  }

  public boolean WithinTolerance(StructureState target) {
    var elevatorReady =
        elevator.isReady(
            target.ElevatorHeight(),
            config.SetpointElevatorTolerance(),
            MetersPerSecond.zero(),
            config.SetpointElevatorVelocityTolerance());
    var armReady =
        arm.isReady(
            target.ArmPitch(),
            config.SetpointArmTolerance(),
            RotationsPerSecond.zero(),
            config.SetpointArmVelocityTolerance());

    RJLog.log("Elevator/IsAtSetpoint", elevatorReady);
    RJLog.log("AlgaeArm/IsAtSetpoint", armReady);

    SmartDashboard.putBoolean("Elevator/IsAtSetpoint", elevatorReady);
    SmartDashboard.putBoolean("AlgaeArm/IsAtSetpoint", armReady);

    return elevatorReady && armReady;
  }
}
