package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.ClimberConfig;
import java.util.function.Supplier;

public class Climber extends SubsystemBase {
  private final TalonFX Climber;
  private final ClimberConfig config;
  private final VoltageOut manualControl = new VoltageOut(0);
  private PositionVoltage positionControl = new PositionVoltage(0);
  private final MotionMagicVoltage motionMagicControl = new MotionMagicVoltage(0);
  private final StatusSignal<Angle> positionSignal;

  private final Alert missingMechanismAlert = new Alert("Climber is Degraded", AlertType.kError);

  public Climber(ClimberConfig config) {
    this.config = config;
    this.Climber = config.MotorID().getMotor();
    this.Climber.getConfigurator().apply(config.MotorConfig());

    positionSignal = Climber.getPosition();
    Climber.setPosition(0);
    setDefaultCommand(idleCommand());
  }

  public Command ClimberDeploy() {
    return run(() -> deploy()).withName("Climber::DeployWithButtonPress");
  }

  private void deploy() {
    Climber.setControl(motionMagicControl.withPosition((config.ClimberDeploy())));
  }

  public Command idleCommand() {
    return run(this::idle).withName("Climber::Idle");
  }

  public Command manualCommand(Supplier<Double> manualInput) {
    return run(() -> manual(manualInput.get())).withName("Climber::Manual");
  }

  private void idle() {
    Climber.setControl(manualControl.withOutput(0));
  }

  public void resetPosition() {
    Climber.setPosition(0);
  }

  private void manual(double output) {
    Climber.setControl(manualControl.withOutput(Volts.of(12).times(output)));
  }

  public Command goToPositionCommand(Angle angle) {
    return run(() -> goToPosition(angle)).withName("Climber::MoveToPosition");
  }

  public Command HoldPosition() {
    return run(() -> goToPosition(getPosition())).withName("Climber::HoldPosition");
  }

  public void goToPosition(Angle target) {
    Climber.setControl(positionControl.withPosition(target));
  }

  public Angle getPosition() {
    return positionSignal.refresh().getValue();
  }

  public void refreshAlert() {
    boolean ClimberOK = Climber.isConnected(); // && Climber.getFaultField().getValue() == 0;
    missingMechanismAlert.set(!ClimberOK);
  }
}
