package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static frc.robot.oi.StructureState.*;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.constants.Constants.ElevatorConfig;
import frc.robot.oi.StructureState;

public class Elevator extends LinearMechanism {

  private final MotionMagicExpoTorqueCurrentFOC elevatorControl =
      new MotionMagicExpoTorqueCurrentFOC(0).withSlot(1);
  private final MotionMagicVoltage slot0Control = new MotionMagicVoltage(0);

  private final ElevatorConfig config;

  private final TalonFX secondaryMotor;
  private final Follower followRequest = new Follower(0, false);

  private final Alert missingMechanismAlert = new Alert("Elevator is Degraded", AlertType.kError);

  public Elevator(ElevatorConfig config) {
    super(
        config.MotorID(),
        config.MotorConfig(),
        config.MechanismConfig(),
        config.SimConfig(),
        config.SysIdConfig());
    this.config = config;

    this.secondaryMotor = config.SecondaryMotorID().getMotor();
    this.secondaryMotor.getConfigurator().apply(config.MotorConfig());
    this.secondaryMotor.setControl(
        followRequest.withMasterID(config.MotorID().ID()).withOpposeMasterDirection(true));
  }

  public Command SetpointCommand(StructureState target) {
    return goToPositionCommand(target::ElevatorHeight)
        .withName(String.format("Elevator::%s", target));
  }

  public Command HomeCommand() {
    return SetpointCommand(HOME);
  }

  public Command IntakeCoralCommand() {
    return SetpointCommand(INTAKE_CORAL);
  }

  public Command ScoreCoralL1Command() {
    return SetpointCommand(SCORE_CORAL_L1);
  }

  public Command ScoreCoralL2Command() {
    return SetpointCommand(SCORE_CORAL_L2);
  }

  public Command ScoreCoralL3Command() {
    return SetpointCommand(SCORE_CORAL_L3);
  }

  public Command ScoreCoralL4Command() {
    return SetpointCommand(SCORE_CORAL_L4);
  }

  public Command IntakeAlgaeFloorCommand() {
    return SetpointCommand(INTAKE_ALGAE_FLOOR);
  }

  public Command ScoreAlgaeProcessorCommand() {
    return SetpointCommand(SCORE_ALGAE_PROCESSOR);
  }

  public Command IntakeAlgaeL2Command() {
    return SetpointCommand(INTAKE_ALGAE_L2);
  }

  public Command IntakeAlgaeL3Command() {
    return SetpointCommand(INTAKE_ALGAE_L3);
  }

  public Command ScoreAlgaeBargeCommand() {
    return SetpointCommand(SCORE_ALGAE_BARGE);
  }

  public Command ClimbCommand() {
    return SetpointCommand(CLIMB);
  }

  @Override
  public Command zeroCommand() {
    var zero = super.zeroCommand();

    return zero.andThen(new ScheduleCommand(HomeCommand())).withName(zero.getName());
  }

  @Override
  protected ControlRequest closedLoopRequest(Distance target) {
    return elevatorControl.withPosition(target.in(Meters) / config.MechanismConfig().DrumRatio());
  }

  public Command sysIdRoutine() {
    return sysIdQuasistatic(Direction.kForward, () -> getPosition().gt(config.SysIdMax()))
        .andThen(sysIdQuasistatic(Direction.kReverse, () -> getPosition().lt(config.SysIdMin())))
        .andThen(sysIdDynamic(Direction.kForward, () -> getPosition().gt(config.SysIdMax())))
        .andThen(sysIdDynamic(Direction.kReverse, () -> getPosition().lt(config.SysIdMin())))
        .withName("Elevator::SysIdRoutine");
  }

  public void setBrakeMode() {
    motor
        .getConfigurator()
        .apply(motorConfig.MotorOutput.withNeutralMode(NeutralModeValue.Brake), 0);
  }

  public void setCoastMode() {
    motor
        .getConfigurator()
        .apply(motorConfig.MotorOutput.withNeutralMode(NeutralModeValue.Coast), 0);
  }

  public void refreshAlert() {
    boolean ElevatorFollowerHealthy =
        secondaryMotor.isConnected(); // && secondaryMotor.getFaultField().getValue() == 0;
    boolean ElevatorLeaderHealthy =
        motor.isConnected(); // && secondaryMotor.getFaultField().getValue() == 0;
    boolean OverallHealth = ElevatorLeaderHealthy && ElevatorFollowerHealthy;
    missingMechanismAlert.set(!OverallHealth);
  }
}
