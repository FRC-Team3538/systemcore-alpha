package frc.robot.subsystems;

import static edu.wpi.first.wpilibj2.command.Commands.either;
import static edu.wpi.first.wpilibj2.command.Commands.select;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static frc.robot.oi.StructureState.SCORE_CORAL_L1;
import static frc.robot.oi.StructureState.SCORE_CORAL_L2;
import static frc.robot.oi.StructureState.SCORE_CORAL_L3;
import static frc.robot.oi.StructureState.SCORE_CORAL_L4;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.CoralMechConfig;
import frc.robot.lib.RJLog;
import frc.robot.oi.StructureState;
import java.util.Map;
import java.util.function.Supplier;

public class CoralMech extends SubsystemBase {
  private final CoralMechConfig config;
  private final TalonFX motor;
  private final CANdi candi;
  private boolean coralLoaded;
  private boolean coralDetected;
  private boolean hadCoral;

  private final VoltageOut controlRequest = new VoltageOut(0);
  private Alert missingMechanismAlert = new Alert("Coral Mech is Degraded", AlertType.kError);

  public CoralMech(CoralMechConfig config) {
    this.config = config;
    this.motor = config.MotorID().getMotor();
    this.motor.getConfigurator().apply(config.motorconfig());
    this.candi = config.candiID().getCANdi();
    this.candi.getConfigurator().apply(config.candiconfig());

    coralLoaded = coralDetected();
    setDefaultCommand(Stop());
  }

  public Command RunCoralPower(Supplier<Double> PercentPower) {
    return run(() -> motor.setControl(controlRequest.withOutput(PercentPower.get() * 12)))
        .withName("CoralMech::RunCoralPower");
  }

  public Command RunCoralVoltage(Supplier<Double> Voltage) {
    return run(() -> motor.setControl(controlRequest.withOutput(Voltage.get())))
        .withName("CoralMech::RunCoralPower");
  }

  public Command Stop() {
    return run(motor::stopMotor).withName("CoralMech::Stop");
  }

  public Command Intake() {

    return sequence(
            runOnce(
                () -> {
                  coralLoaded = false;
                }),
            RunCoralPower(config::IntakePower).until(this::coralDetected),
            RunCoralPower(config::IntakePower).withTimeout(config.IntakeDelay()),
            runOnce(
                () -> {
                  coralLoaded = true;
                  motor.stopMotor();
                }))
        .withName("CoralMech::Intake");
  }

  public Command Score(Supplier<StructureState> currentConfiguration) {
    return ScoreByStructureState(currentConfiguration)
        .until(() -> !coralDetected())
        .andThen(
            runOnce(
                () -> {
                  coralLoaded = false;
                }))
        .andThen(ScoreByStructureStateWithDelay(currentConfiguration))
        .handleInterrupt(() -> coralLoaded = coralDetected())
        .withName("CoralMech::Score");
  }

  private Command ScoreByStructureState(Supplier<StructureState> currentConfiguration) {
    return either(
        select(
            Map.of(
                SCORE_CORAL_L1, RunCoralPower(config::L1ScorePower),
                SCORE_CORAL_L2, RunCoralPower(config::L2ScorePower),
                SCORE_CORAL_L3, RunCoralPower(config::L3ScorePower),
                SCORE_CORAL_L4, RunCoralPower(config::L4ScorePower)),
            currentConfiguration),
        RunCoralPower(config::DefaultScorePower),
        () -> currentConfiguration.get().isScoreCoralConfig());
  }

  private Command ScoreByStructureStateWithDelay(Supplier<StructureState> currentConfiguration) {
    return either(
        select(
            Map.of(
                SCORE_CORAL_L1,
                    RunCoralPower(config::L1ScorePower).withTimeout(config.L1ScoreDelay()),
                SCORE_CORAL_L2,
                    RunCoralPower(config::L2ScorePower).withTimeout(config.L2ScoreDelay()),
                SCORE_CORAL_L3,
                    RunCoralPower(config::L3ScorePower).withTimeout(config.L3ScoreDelay()),
                SCORE_CORAL_L4,
                    RunCoralPower(config::L4ScorePower).withTimeout(config.L4ScoreDelay())),
            currentConfiguration),
        RunCoralPower(config::DefaultScorePower).withTimeout(config.DefaultScoreDelay()),
        () -> currentConfiguration.get().isScoreCoralConfig());
  }

  public boolean coralDetected() {
    return coralDetected;
  }

  public boolean hasCoral() {
    return coralLoaded;
  }

  public boolean lostCoral() {
    return hadCoral && !coralDetected;
  }

  @Override
  public void periodic() {
    super.periodic();

    hadCoral = coralDetected;
    coralDetected = candi.getS1Closed().getValue();

    if (DriverStation.isDisabled()) {
      coralLoaded = coralDetected;
    }

    RJLog.log("CoralMech/CoralDetected", coralDetected());
    RJLog.log("CoralMech/CoralLoaded", hasCoral());

    SmartDashboard.putBoolean("CoralMech/CoralDetected", coralDetected());
    SmartDashboard.putBoolean("CoralMech/CoralLoaded", hasCoral());
  }

  public void refreshAlert() {
    var CoralMechisHealthy = motor.isConnected(); // && motor.getFaultField().getValue() == 0;

    var candiIsHealthy = candi.isConnected() && candi.getFaultField().getValue() == 0;

    missingMechanismAlert.set(!(CoralMechisHealthy && candiIsHealthy));
  }
}
