package frc.robot.subsystems;

import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.Constants.AlgaeMechConfig;
import frc.robot.lib.RJLog;
import java.util.function.Supplier;

public class AlgaeMech extends SubsystemBase {
  private final TalonFX motor;
  private final AlgaeMechConfig config;
  private final DutyCycleOut controlRequest = new DutyCycleOut(0);
  private final double acquireThreshold = 55.0;
  private final double releaseThreshold = 35.0;
  private boolean algaeLoaded = false;

  private Alert missingMechanismAlert = new Alert("Algae Mech is Degraded", AlertType.kError);

  public AlgaeMech(AlgaeMechConfig config) {
    this.config = config;
    this.motor = config.MotorID().getMotor();
    this.motor.getConfigurator().apply(config.MotorConfig());
    setDefaultCommand(Idle());

    new Trigger(() -> this.algaeDetected(acquireThreshold))
        .debounce(0.5, DebounceType.kFalling)
        .onTrue(
            Commands.runOnce(
                () -> {
                  algaeLoaded = true;
                }));

    new Trigger(() -> this.algaeDetected(releaseThreshold))
        .debounce(0.5, DebounceType.kFalling)
        .onFalse(
            Commands.runOnce(
                () -> {
                  algaeLoaded = false;
                }));
  }

  public Command RunAlgaePower(Supplier<Double> PercentPower) {
    return run(() -> motor.setControl(controlRequest.withOutput(PercentPower.get())))
        .withName("AlgaeMech::RunAlgaePower");
  }

  public Command StopAlgae() {
    return run(motor::stopMotor).withName("AlgaeMech::Stop");
  }

  public Command Idle() {
    return run(() -> {
          if (algaeLoaded) {
            motor.setControl(controlRequest.withOutput(-1));
          } else {
            motor.stopMotor();
          }
        })
        .withName("AlgaeMech::Idle");
  }

  public Command Intake() {
    return RunAlgaePower(config::IntakePower)
        .until(this::algaeDetected)
        .withName("AlgaeMech::Intake");
  }

  public Command AutonScore() {
    return RunAlgaePower(config::ScorePower)
        .withTimeout(config.ScoreDelay())
        .alongWith(waitSeconds(0.25).andThen(Commands.runOnce(() -> algaeLoaded = false)))
        .withName("AlgaeMech::AutonScore");
  }

  public Command Score() {
    return RunAlgaePower(config::ScorePower)
        .withTimeout(config.ScoreDelay())
        .withName("AlgaeMech::Score");
  }

  private boolean algaeDetected() {
    return algaeDetected(acquireThreshold);
  }

  private boolean algaeDetected(double threshold) {
    double statorCurrent = motor.getTorqueCurrent().getValueAsDouble();
    boolean detected = Math.abs(statorCurrent) > threshold;
    return detected;
  }

  public boolean HoldsAlgae() {
    return algaeLoaded;
  }

  @Override
  public void periodic() {
    super.periodic();

    RJLog.log("AlgaeMech/HoldsAlgae", HoldsAlgae());
    // RJLog.log("AlgaeMech/AlgaeDetected", algaeDetected());

    SmartDashboard.putBoolean("AlgaeMech/HoldsAlgae", HoldsAlgae());
    SmartDashboard.putBoolean("AlgaeMech/AlgaeDetected", algaeDetected());
  }

  public void refreshAlert() {
    var AlgaeMotorHealthy = motor.isConnected(); // && motor.getFaultField().getValue() == 0;
    missingMechanismAlert.set(!AlgaeMotorHealthy);
  }
}
