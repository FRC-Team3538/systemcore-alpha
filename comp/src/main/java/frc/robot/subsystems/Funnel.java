package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.Constants.FunnelConfig;
import frc.robot.lib.RJLog;
import java.util.function.Supplier;

public class Funnel extends SubsystemBase {
  private final TalonFX Funnel;
  private final PWMVictorSPX FunnelSolenoid;
  private final FunnelConfig config;
  private final DutyCycleOut controlRequest = new DutyCycleOut(0);
  private final CANrange canRange;
  private Alert missingMechanismAlert = new Alert("Funnel is Degraded", AlertType.kError);

  public Funnel(FunnelConfig config) {
    this.config = config;
    this.Funnel = config.MotorID().getMotor();
    this.Funnel.getConfigurator().apply(config.MotorConfig());
    this.canRange = config.CANrangeID().getCANrange();
    this.canRange.getConfigurator().apply(config.CANrangeConfig());
    this.FunnelSolenoid = config.FunnelSolenoidID().getVictorSPX();
    setDefaultCommand(Stop());
  }

  public Command FunnelPower(Supplier<Double> PercentPower) {
    return run(() -> Funnel.setControl(controlRequest.withOutput(PercentPower.get())))
        .withName("Funnel::FunnelPower");
  }

  public Command Stop() {
    return run(() -> {
          Funnel.stopMotor();
          FunnelSolenoid.set(config.standbyPower());
        })
        .withName("Funnel::Stop");
  }

  public Command Intake() {
    return FunnelPower(config::IntakePower).withName("Funnel::Intake");
  }

  public Command Outake() {
    return FunnelPower(config::OutakePower).withName("Funnel::Outake");
  }

  public boolean DetectCoral() {
    return canRange.getIsDetected().getValue();
  }

  public Trigger CoralInFeeder() {
    var threshold = Amps.of(30);
    return new Trigger(() -> Funnel.getTorqueCurrent().getValue().gt(threshold))
        .debounce(0.05, DebounceType.kRising);
  }

  public Command FunnelSolenoidDeploy() {
    return run(() -> FunnelSolenoid.set(config.actuatePower()))
        .withName("FunnelSolenoid::DeployFunnel");
  }

  public Command FunnelSolenoidStop() {
    return run(() -> FunnelSolenoid.set(config.standbyPower())).withName("FunnelSolenoid::Standby");
  }

  // Old Trigger
  public Trigger CoralInFeederWithoutDebounce() {
    var threshold = Amps.of(20);
    return new Trigger(() -> Funnel.getTorqueCurrent().getValue().gt(threshold));
  }

  @Override
  public void periodic() {
    RJLog.log("Funnel/CoralDetected", DetectCoral());
    SmartDashboard.putBoolean("Funnel/CoralDetected", DetectCoral());
  }

  public void refreshAlert() {
    var FunnelMotorIsHealthy = Funnel.isConnected(); // && Funnel.getFaultField().getValue() == 0;

    var CANrangeIsHealthy = canRange.isConnected(); // && canRange.getFaultField().getValue() == 0;

    missingMechanismAlert.set(!(FunnelMotorIsHealthy && CANrangeIsHealthy));
  }
}
