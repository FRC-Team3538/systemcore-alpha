package frc.robot.subsystems;

import static frc.robot.oi.StructureState.*;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.constants.Constants.AlgaeArmConfig;
import frc.robot.oi.StructureState;

public class AlgaeArm extends AngularMechanism {
  private final AlgaeArmConfig config;

  private final MotionMagicVoltage armControl = new MotionMagicVoltage(0);

  private final Alert missingMechanismAlert = new Alert("Algae Arm is Degraded", AlertType.kError);

  public AlgaeArm(AlgaeArmConfig config) {
    super(
        config.MotorID(),
        config.MotorConfig(),
        config.MechanismConfig(),
        config.SimConfig(),
        config.SysIdConfig());
    this.config = config;
  }

  public Command SetpointCommand(StructureState target) {
    return goToPositionCommand(target::ArmPitch).withName(String.format("AlgaeArm::%s", target));
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
  public Command zeroForwardCommand() {
    var zero = super.zeroForwardCommand();

    return zero.andThen(new ScheduleCommand(HomeCommand())).withName(zero.getName());
  }

  @Override
  protected ControlRequest closedLoopRequest(Angle target) {
    return armControl.withPosition(target);
  }

  public Command sysIdRoutine() {
    return sysIdQuasistatic(Direction.kReverse, () -> getPosition().lt(config.SysIdMin()))
        .andThen(sysIdQuasistatic(Direction.kForward, () -> getPosition().gt(config.SysIdMax())))
        .andThen(sysIdDynamic(Direction.kReverse, () -> getPosition().lt(config.SysIdMin())))
        .andThen(sysIdDynamic(Direction.kForward, () -> getPosition().gt(config.SysIdMax())))
        .withName("AlgaeArm::SysIdRoutine");
  }

  public void refreshAlert() {
    var healthy = motor.isConnected(); // && motor.getFaultField().getValue() == 0;

    missingMechanismAlert.set(!healthy);
  }
}
