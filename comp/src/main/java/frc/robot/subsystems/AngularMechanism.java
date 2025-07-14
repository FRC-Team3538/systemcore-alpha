package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.wpilibj2.command.Commands.print;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.constants.Constants.AngularMechanismConfig;
import frc.robot.constants.Constants.AngularSimConfig;
import frc.robot.constants.Constants.TalonFXID;
import frc.robot.lib.CANSignalManager;
import frc.robot.lib.RJLog;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public abstract class AngularMechanism extends SubsystemBase {
  protected final TalonFX motor;
  private final TalonFXConfiguration motorConfig;

  private final StatusSignal<Angle> positionSignal;
  private final StatusSignal<AngularVelocity> velocitySignal;
  private final StatusSignal<Current> torqueCurrentSignal;
  private final StatusSignal<Double> closedLoopReferenceSignal;

  private final SysIdRoutine sysIdRoutine;

  private final Mechanism2d armMechanism;
  private final MechanismLigament2d arm;

  private final AngularMechanismConfig config;
  private final SingleJointedArmSim armSim;

  private final ArmFeedforward feedforward;
  private VoltageOut manualRequest = new VoltageOut(0);
  private VoltageOut sysIdRequest = new VoltageOut(0);

  private boolean zeroed = false;

  public AngularMechanism(
      TalonFXID motorID,
      TalonFXConfiguration motorConfig,
      AngularMechanismConfig mechanismConfig,
      AngularSimConfig simConfig,
      SysIdRoutine.Config sysIdConfig) {
    this.config = mechanismConfig;
    this.motor = motorID.getMotor();
    this.motorConfig = motorConfig;

    motor.getConfigurator().apply(motorConfig);

    feedforward =
        new ArmFeedforward(
            motorConfig.Slot0.kS, motorConfig.Slot0.kG, motorConfig.Slot0.kV, motorConfig.Slot0.kA);

    positionSignal = motor.getPosition();
    velocitySignal = motor.getVelocity();
    torqueCurrentSignal = motor.getTorqueCurrent();
    closedLoopReferenceSignal = motor.getClosedLoopReference();

    CANSignalManager.registerSignals(
        motorID.CANBus(),
        positionSignal,
        velocitySignal,
        torqueCurrentSignal,
        closedLoopReferenceSignal);

    armSim =
        new SingleJointedArmSim(
            LinearSystemId.identifyPositionSystem(motorConfig.Slot0.kV, motorConfig.Slot0.kA),
            DCMotor.getKrakenX60(1).withReduction(mechanismConfig.GearboxRatio()),
            1,
            mechanismConfig.ArmLength().in(Meters),
            mechanismConfig.MinPosition().in(Radians),
            mechanismConfig.MaxPosition().in(Radians),
            simConfig.SimulateGravity(),
            simConfig.InitialPosition().in(Radians),
            simConfig.PositionStdDev().in(Radians),
            simConfig.VelocityStdDev().in(RadiansPerSecond));

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

    armMechanism =
        new Mechanism2d(
            config.ArmLength().times(2).in(Meters), config.ArmLength().times(2).in(Meters));
    var root =
        armMechanism.getRoot(
            "Base", config.ArmLength().div(2).in(Meters), config.ArmLength().div(2).in(Meters));
    arm = new MechanismLigament2d("Carriage", config.ArmLength().in(Meters), 0);
    root.append(arm);

    SmartDashboard.putData(String.format("Mechanisms/%s", getName()), armMechanism);
  }

  public Command goToPositionCommand(Angle target) {
    return run(() -> goToPosition(target))
        .withName(
            String.format(String.format("%s::GoToPosition(%s)", getName(), target.in(Degrees))));
  }

  public Command goToPositionCommand(Supplier<Angle> target) {
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

  private void goToPosition(Angle target) {
    manualRequest.Output = 0;

    applyRequest(closedLoopRequest(target));
  }

  private void manual(double manual) {
    applyRequest(
        manualRequest.withOutput(
            Volts.of(12)
                .times(manual)
                .plus(Volts.of(feedforward.calculate(getPosition().in(Radians), 0)))));
  }

  private void applyRequest(ControlRequest request) {
    motor.setControl(request);
  }

  protected abstract ControlRequest closedLoopRequest(Angle target);

  public Angle getPosition() {
    return positionSignal.getValue();
  }

  public AngularVelocity getVelocity() {
    return velocitySignal.getValue();
  }

  public Angle getTargetPosition() {
    return Rotations.of(closedLoopReferenceSignal.getValue());
  }

  public boolean isReady(Angle tolerance) {
    return isReady(getTargetPosition());
  }

  public boolean isReady(Angle target, Angle tolerance) {
    return isReady(target, tolerance, RotationsPerSecond.zero(), RotationsPerSecond.zero());
  }

  public boolean isReady(
      Angle target,
      Angle positionTolerance,
      AngularVelocity targetVelocity,
      AngularVelocity velocityTolerance) {

    RJLog.log(String.format("%s/IsReady/Manual", getName()), manualRequest.Output);
    RJLog.log(
        String.format("%s/IsReady/PositionError", getName()),
        getPosition().minus(target).abs(Degrees));
    RJLog.log(
        String.format("%s/IsReady/VelocityError", getName()),
        getVelocity().minus(targetVelocity).abs(DegreesPerSecond));
    RJLog.log(
        String.format("%s/IsReady/PositionTolerance", getName()), positionTolerance.in(Degrees));
    RJLog.log(
        String.format("%s/IsReady/VelocityTolerance", getName()),
        velocityTolerance.in(DegreesPerSecond));

    return manualRequest.Output != 0
        || (getPosition().minus(target).abs(Rotations) <= positionTolerance.in(Rotations)
            && getVelocity().minus(targetVelocity).abs(RotationsPerSecond)
                <= velocityTolerance.in(RotationsPerSecond));
  }

  @Override
  public void periodic() {
    arm.setAngle(getPosition().in(Degrees));
    RJLog.log(String.format("%s/Zeroed", getName()), zeroed);
  }

  public ControlRequest getlatestRequest() {
    return motor.getAppliedControl();
  }

  public boolean zeroed() {
    return zeroed;
  }

  public Command zeroCommand() {
    return Commands.wait(0.1)
        .andThen(
            waitUntil(
                () ->
                    velocitySignal.getValue().gt(RotationsPerSecond.of(-0.1))
                        && torqueCurrentSignal.getValue().lt(Amps.of(-20))))
        .deadlineFor(
            manualCommand(() -> -0.2),
            print(String.format("Zeroing %s", getName())),
            Commands.runOnce(
                () ->
                    motor
                        .getConfigurator()
                        .apply(
                            motorConfig.SoftwareLimitSwitch.withReverseSoftLimitEnable(false), 0)))
        // .traced("ApplyMotorConfig"))
        .andThen(
            Commands.runOnce(
                () -> {
                  motor.setPosition(config.MinPosition().in(Rotations), 0);
                  zeroed = true;
                  motor.stopMotor();
                }),
            // .traced("FinalizeZero"),
            print(String.format("Zeroed %s", getName())))
        .finallyDo(
            () ->
                // Tracer.traceFunc(
                //     "ApplyMotorConfig",
                //     () ->
                motor
                    .getConfigurator()
                    .apply(motorConfig.SoftwareLimitSwitch.withReverseSoftLimitEnable(true), 0))
        .withName(String.format("%s::Zero(%s)", getName(), Direction.kReverse));
  }

  public Command zeroForwardCommand() {
    return Commands.wait(0.1)
        .andThen(
            waitUntil(
                () ->
                    velocitySignal.getValue().lt(RotationsPerSecond.of(0.1))
                        && torqueCurrentSignal.getValue().gt(Amps.of(20))))
        .deadlineFor(
            manualCommand(() -> 0.2),
            print(String.format("Zeroing %s", getName())),
            Commands.runOnce(
                () ->
                    motor
                        .getConfigurator()
                        .apply(
                            motorConfig.SoftwareLimitSwitch.withForwardSoftLimitEnable(false), 0)))
        // .traced("ApplyMotorConfig"))
        .andThen(
            Commands.runOnce(
                () -> {
                  motor.setPosition(config.MaxPosition().in(Rotations), 0);
                  zeroed = true;
                  motor.stopMotor();
                }),
            // .traced("FinalizeZero"),
            print(String.format("Zeroed %s", getName())))
        .finallyDo(
            () ->
                // Tracer.traceFunc(
                //     "ApplyMotorConfig",
                //     () ->
                motor
                    .getConfigurator()
                    .apply(motorConfig.SoftwareLimitSwitch.withForwardSoftLimitEnable(true), 0))
        .withName(String.format("%s::Zero(%s)", getName(), Direction.kForward));
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

    armSim.setInputVoltage(voltage.in(Volts));
    armSim.update(0.02);

    simState.setForwardLimit(armSim.hasHitUpperLimit());
    simState.setReverseLimit(armSim.hasHitLowerLimit());

    simState.setRawRotorPosition(
        Units.radiansToRotations(armSim.getAngle()) / config.GearboxRatio());
    simState.setRotorVelocity(
        Units.radiansToRotations(armSim.getVelocity()) / config.GearboxRatio());
  }
}
