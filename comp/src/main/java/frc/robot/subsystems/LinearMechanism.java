package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.wpilibj2.command.Commands.*;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj3.Tracer;
import frc.robot.constants.Constants.LinearMechanismConfig;
import frc.robot.constants.Constants.LinearSimConfig;
import frc.robot.constants.Constants.TalonFXID;
import frc.robot.lib.CANSignalManager;
import frc.robot.lib.RJLog;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public abstract class LinearMechanism extends SubsystemBase {
  protected final TalonFX motor;
  protected final TalonFXConfiguration motorConfig;

  private final StatusSignal<Angle> positionSignal;
  private final StatusSignal<AngularVelocity> velocitySignal;
  private final StatusSignal<Double> closedLoopReferenceSignal;

  private final SysIdRoutine sysIdRoutine;

  private final Mechanism2d elevatorMechanism;
  private final MechanismLigament2d carriage;

  private final LinearMechanismConfig config;
  private final ElevatorSim elevatorSim;

  private final ElevatorFeedforward feedforward;
  private VoltageOut manualRequest = new VoltageOut(0);
  private VoltageOut sysIdRequest = new VoltageOut(0);

  private boolean zeroed = false;

  public LinearMechanism(
      TalonFXID motorID,
      TalonFXConfiguration motorConfig,
      LinearMechanismConfig mechanismConfig,
      LinearSimConfig simConfig,
      SysIdRoutine.Config sysIdConfig) {
    this.config = mechanismConfig;
    this.motor = motorID.getMotor();
    this.motorConfig = motorConfig;

    motor.getConfigurator().apply(motorConfig);

    feedforward =
        new ElevatorFeedforward(
            motorConfig.Slot0.kS,
            motorConfig.Slot0.kG,
            motorConfig.Slot0.kV / mechanismConfig.DrumRatio(),
            motorConfig.Slot0.kA / mechanismConfig.DrumRatio());

    positionSignal = motor.getPosition();
    velocitySignal = motor.getVelocity();
    closedLoopReferenceSignal = motor.getClosedLoopReference();

    CANSignalManager.registerSignals(
        motorID.CANBus(), positionSignal, velocitySignal, closedLoopReferenceSignal);

    elevatorSim =
        new ElevatorSim(
            LinearSystemId.identifyPositionSystem(
                motorConfig.Slot0.kV / mechanismConfig.DrumRatio(),
                motorConfig.Slot0.kA / mechanismConfig.DrumRatio()),
            DCMotor.getKrakenX60(1)
                .withReduction(mechanismConfig.GearboxRatio())
                .withReduction(mechanismConfig.DrumRatio()),
            mechanismConfig.MinPosition().in(Meters),
            mechanismConfig.MaxPosition().in(Meters),
            simConfig.SimulateGravity(),
            simConfig.InitialPosition().in(Meters),
            simConfig.PositionStdDev().in(Meters),
            simConfig.VelocityStdDev().in(MetersPerSecond));

    sysIdRoutine =
        new SysIdRoutine(
            sysIdConfig,
            new SysIdRoutine.Mechanism(
                volts -> {
                  motor.setControl(sysIdRequest.withOutput(volts));
                },
                null,
                this,
                getName()));

    elevatorMechanism =
        new Mechanism2d(config.MaxPosition().in(Meters) / 10.0, config.MaxPosition().in(Meters));
    var root =
        elevatorMechanism.getRoot(
            "Base", config.MaxPosition().in(Meters) / 20.0, config.MinPosition().in(Meters));
    carriage = new MechanismLigament2d("Carriage", 0, 90);
    root.append(carriage);

    SmartDashboard.putData(String.format("Mechanisms/%s", getName()), elevatorMechanism);
  }

  public Command goToPositionCommand(Distance target) {
    return run(() -> goToPosition(target))
        .withName(
            String.format(String.format("%s::GoToPosition(%s)", getName(), target.in(Meters))));
  }

  public Command goToPositionCommand(Supplier<Distance> target) {
    return run(() -> goToPosition(target.get()))
        .withName(String.format("%s::GoToPosition", getName()));
  }

  public Command manualCommand(Supplier<Double> manualInput) {
    return run(() -> manual(manualInput.get())).withName(String.format("%s::Manual", getName()));
  }

  public Command goToCurrentPositionCommand() {
    return defer(() -> goToPositionCommand(getPosition()))
        .withName(String.format("%s::GoToPosition(Current)", getName()));
  }

  private void goToPosition(Distance target) {
    manualRequest.Output = 0;
    applyRequest(closedLoopRequest(target));
  }

  private void manual(double manual) {
    applyRequest(
        manualRequest.withOutput(
            Volts.of(12).times(manual).plus(Volts.of(feedforward.calculate(0)))));
  }

  private void applyRequest(ControlRequest request) {
    motor.setControl(request);
  }

  protected abstract ControlRequest closedLoopRequest(Distance target);

  public Distance getPosition() {
    return Meters.of(positionSignal.getValue().in(Rotations) * config.DrumRatio());
  }

  public LinearVelocity getVelocity() {
    return MetersPerSecond.of(
        velocitySignal.getValue().in(RotationsPerSecond) * config.DrumRatio());
  }

  public Distance getTargetPosition() {
    return Meters.of(closedLoopReferenceSignal.getValue() * config.DrumRatio());
  }

  public boolean isReady(Distance positionTolerance, LinearVelocity velocityTolerance) {
    return isReady(
        getTargetPosition(), positionTolerance, MetersPerSecond.zero(), velocityTolerance);
  }

  public boolean isReady(Distance tolerance) {
    return isReady(getTargetPosition(), tolerance);
  }

  public boolean isReady(
      Distance target,
      Distance positionTolerance,
      LinearVelocity targetVelocity,
      LinearVelocity velocityTolerance) {
    RJLog.log(String.format("%s/IsReady/Manual", getName()), manualRequest.Output);
    RJLog.log(
        String.format("%s/IsReady/PositionError", getName()),
        getPosition().minus(target).abs(Inches));
    RJLog.log(
        String.format("%s/IsReady/VelocityError", getName()),
        getVelocity().minus(targetVelocity).abs(InchesPerSecond));
    RJLog.log(
        String.format("%s/IsReady/PositionTolerance", getName()), positionTolerance.in(Inches));
    RJLog.log(
        String.format("%s/IsReady/VelocityTolerance", getName()),
        velocityTolerance.in(InchesPerSecond));

    var output =
        manualRequest.Output != 0
            || (getPosition().minus(target).abs(Meters) <= positionTolerance.in(Meters)
                && getVelocity().minus(targetVelocity).abs(MetersPerSecond)
                    <= velocityTolerance.in(MetersPerSecond));

    RJLog.log(String.format("%s/IsReady", getName()), output);

    return output;
  }

  public boolean isReady(Distance target, Distance tolerance) {
    return isReady(target, tolerance, MetersPerSecond.zero(), MetersPerSecond.zero());
  }

  @Override
  public void periodic() {
    carriage.setLength(getPosition().in(Meters));
  }

  public ControlRequest getlatestRequest() {
    return motor.getAppliedControl();
  }

  public boolean zeroed() {
    return zeroed;
  }

  public Command zeroCommand() {
    return waitSeconds(0.1)
        .andThen(waitUntil(() -> velocitySignal.getValue().gt(RotationsPerSecond.of(-0.1))))
        .deadlineFor(
            manualCommand(() -> -0.2),
            print(String.format("Zeroing %s", getName())),
            Commands.runOnce(
                    () ->
                        motor
                            .getConfigurator()
                            .apply(
                                motorConfig.SoftwareLimitSwitch.withReverseSoftLimitEnable(false),
                                0))
                .traced("ApplyMotorConfig"))
        .traced("pulldown")
        .andThen(
            Commands.runOnce(
                    () -> {
                      motor.setPosition(config.MinPosition().in(Meters) / config.DrumRatio(), 0);
                      zeroed = true;
                      motor.stopMotor();
                    })
                .traced("FinalizeZero"),
            print(String.format("Zeroed %s", getName())))
        .finallyDo(
            () ->
                Tracer.traceFunc(
                    "ApplyMotorConfig",
                    () ->
                        motor
                            .getConfigurator()
                            .apply(
                                motorConfig.SoftwareLimitSwitch.withReverseSoftLimitEnable(true),
                                0)))
        .withName(String.format("%s::Zero", getName()));
  }

  public Command sysIdQuasistatic(Direction direction, BooleanSupplier condition) {
    return sysIdRoutine.quasistatic(direction).until(condition);
  }

  public Command sysIdDynamic(Direction direction, BooleanSupplier condition) {
    return sysIdRoutine.dynamic(direction).until(condition);
  }

  @Override
  public void simulationPeriodic() {
    var simState = motor.getSimState();

    var voltage = simState.getMotorVoltageMeasure();

    elevatorSim.setInputVoltage(voltage.in(Volts));
    elevatorSim.update(0.02);

    simState.setForwardLimit(elevatorSim.hasHitUpperLimit());
    simState.setReverseLimit(elevatorSim.hasHitLowerLimit());

    simState.setRawRotorPosition(
        elevatorSim.getPositionMeters() / config.DrumRatio() / config.GearboxRatio());
    simState.setRotorVelocity(
        elevatorSim.getVelocityMetersPerSecond() / config.DrumRatio() / config.GearboxRatio());
  }
}
