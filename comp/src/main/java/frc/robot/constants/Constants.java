package frc.robot.constants;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static java.util.Map.entry;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.CANdiConfiguration;
import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.ProximityParamsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.LinearAccelerationUnit;
import edu.wpi.first.units.VelocityUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.oi.ControlMode;
import frc.robot.oi.StructureState;

public class Constants {

  private static final VelocityUnit<AngularAccelerationUnit> ANGULAR_JERK =
      RotationsPerSecondPerSecond.per(Second);

  private static final VelocityUnit<LinearAccelerationUnit> LINEAR_JERK =
      MetersPerSecondPerSecond.per(Second);

  public static record ControlsConfig(
      LinearVelocity MaxSpeed, AngularVelocity MaxAngularRate, ControlMode DefaultControlMode) {
    public static final ControlsConfig DEFAULT =
        new ControlsConfig(
            TunerConstants.kSpeedAt12Volts, RotationsPerSecond.of(0.75), ControlMode.CYCLING_MODE);

    public ControlsConfig {
      Preferences.initBoolean("Contols/PathToAnyFace", false);
      Preferences.initBoolean("Contols/UseForceField", false);
    }

    public boolean PathToAnyFace() {
      return Preferences.getBoolean("Contols/PathToAnyFace", false);
    }

    public boolean UseForceField() {
      return Preferences.getBoolean("Contols/UseForceField", false);
    }
  }

  public static record SuperstructureConfig(
      InterpolatingDoubleTreeMap MinimumPitchMap,
      Angle _IntermediaryArmTolerance,
      Angle _SetpointArmTolerance,
      AngularVelocity _SetpointArmVelocityTolerance,
      Distance _IntermediaryElevatorTolerance,
      Distance _SetpointElevatorTolerance,
      LinearVelocity _SetpointElevatorVelocityTolerance,
      StructureState DefaultScoreState) {

    public SuperstructureConfig {
      Preferences.initDouble("Superstructure/AutoScoreBumpUpWait", 0.5);
      Preferences.initDouble("Superstructure/AutoScoreBumpUpHeight", 1);
      Preferences.initDouble("Superstructure/AutoScoreBumpUpHeightL1", 3);

      Preferences.initDouble(
          "Superstructure/IntermediaryArmTolerance,", _IntermediaryArmTolerance.in(Degrees));
      Preferences.initDouble(
          "Superstructure/SetpointArmTolerance,", _SetpointArmTolerance.in(Degrees));
      Preferences.initDouble(
          "Superstructure/SetpointArmVelocityTolerance,",
          _SetpointArmVelocityTolerance.in(DegreesPerSecond));
      Preferences.initDouble(
          "Superstructure/IntermediaryElevatorTolerance,",
          _IntermediaryElevatorTolerance.in(Inches));
      Preferences.initDouble(
          "Superstructure/SetpointElevatorTolerance,", _SetpointElevatorTolerance.in(Inches));
      Preferences.initDouble(
          "Superstructure/SetpointElevatorVelocityTolerance,",
          _SetpointElevatorVelocityTolerance.in(InchesPerSecond));
    }

    public Time AutoScoreBumpUpWaitTime() {
      return Seconds.of(Preferences.getDouble("Superstructure/AutoScoreBumpUpWait", 0.5));
    }

    public Distance AutoScoreBumpUpHeightL4() {
      return Inches.of(Preferences.getDouble("Superstructure/AutoScoreBumpUpHeight", 1));
    }

    public Distance AutoScoreBumpUpHeightL1() {
      return Inches.of(Preferences.getDouble("Superstructure/AutoScoreBumpUpHeightL1", 3));
    }

    public Angle MinimumPitchForHeight(Distance height) {
      return Degrees.of(MinimumPitchMap.get(height.in(Inches)));
    }

    public Angle IntermediaryArmTolerance() {
      return Degrees.of(
          Preferences.getDouble(
              "Superstructure/IntermediaryArmTolerance,", _IntermediaryArmTolerance.in(Degrees)));
    }

    public Angle SetpointArmTolerance() {
      return Degrees.of(
          Preferences.getDouble(
              "Superstructure/SetpointArmTolerance,", _SetpointArmTolerance.in(Degrees)));
    }

    public AngularVelocity SetpointArmVelocityTolerance() {
      return DegreesPerSecond.of(
          Preferences.getDouble(
              "Superstructure/SetpointArmVelocityTolerance,",
              _SetpointArmVelocityTolerance.in(DegreesPerSecond)));
    }

    public Distance IntermediaryElevatorTolerance() {
      return Inches.of(
          Preferences.getDouble(
              "Superstructure/IntermediaryElevatorTolerance,",
              _IntermediaryElevatorTolerance.in(Inches)));
    }

    public Distance SetpointElevatorTolerance() {
      return Inches.of(
          Preferences.getDouble(
              "Superstructure/SetpointElevatorTolerance,", _SetpointElevatorTolerance.in(Inches)));
    }

    public LinearVelocity SetpointElevatorVelocityTolerance() {
      return InchesPerSecond.of(
          Preferences.getDouble(
              "Superstructure/SetpointElevatorVelocityTolerance,",
              _SetpointElevatorVelocityTolerance.in(InchesPerSecond)));
    }

    // Map of Height [Inches] to Pitch [Degrees]
    // Pitch = 0deg at horizontal (floor pickup), Pitch = 90deg at vertical (stow)
    public static final SuperstructureConfig DEFAULT =
        new SuperstructureConfig(
            InterpolatingDoubleTreeMap.ofEntries(entry(0.0, 90.0)),
            Degrees.of(60),
            Degrees.of(60),
            DegreesPerSecond.of(60),
            Inches.of(5),
            Inches.of(1),
            InchesPerSecond.of(10),
            StructureState.HOME);
  }

  public static record AlgaeArmConfig(
      TalonFXID MotorID,
      TalonFXConfiguration MotorConfig,
      AngularMechanismConfig MechanismConfig,
      AngularSimConfig SimConfig,
      SysIdRoutine.Config SysIdConfig) {
    public AlgaeArmConfig {
      Preferences.initDouble("AlgaeArm/Stow", 90);
      Preferences.initDouble("AlgaeArm/Receive", 90);
      Preferences.initDouble("AlgaeArm/ScoreBarge", 90);
      Preferences.initDouble("AlgaeArm/ScoreProcessor", 90);
      Preferences.initDouble("AlgaeArm/SysId-Buffer", 5);
    }

    public Angle StowAngle() {
      return Degrees.of(Preferences.getDouble("AlgaeArm/Stow", 90));
    }

    public Angle ReceiveAngle() {
      return Degrees.of(Preferences.getDouble("AlgaeArm/Receive", 90));
    }

    public Angle ScoreBargeAngle() {
      return Degrees.of(Preferences.getDouble("AlgaeArm/ScoreBarge", 90));
    }

    public Angle ScoreProcessorAngle() {
      return Degrees.of(Preferences.getDouble("AlgaeArm/ScoreProcessor", 90));
    }

    public Angle SysIdBuffer() {
      return Degrees.of(Preferences.getDouble("AlgaeArm/SysId-Buffer", 5));
    }

    public Angle SysIdMin() {
      return MechanismConfig.MinPosition.plus(SysIdBuffer());
    }

    public Angle SysIdMax() {
      return MechanismConfig.MaxPosition.minus(SysIdBuffer());
    }

    private static final Double ARM_GEARBOX_RATIO = 40.0 / 1.0;
    // hard stop
    private static final Angle ARM_HARD_MIN = Rotations.zero();
    private static final Angle ARM_SOFT_MIN = ARM_HARD_MIN.plus(Degrees.of(5));

    // couple degrees before upper frame
    private static final Angle ARM_HARD_MAX = Rotations.of(0.4782);
    private static final Angle ARM_SOFT_MAX = ARM_HARD_MAX.minus(Degrees.of(5));

    private static final Distance ARM_LENGTH = Inches.of(24);

    private static final Double kS = 0.22018;
    private static final Double kV = 4.0621;
    private static final Double kA = 0.11542;
    private static final Double kG = 0.19411;
    private static final GravityTypeValue kGravityType = GravityTypeValue.Arm_Cosine;
    private static final Double kP = 63.862;
    private static final Double kD = 6.302;

    public static final AlgaeArmConfig DEFAULT =
        new AlgaeArmConfig(
            new TalonFXID(15, "rio"),
            new TalonFXConfiguration()
                .withSlot0(
                    new Slot0Configs()
                        .withKS(kS)
                        .withKV(kV)
                        .withKA(kA)
                        .withKG(kG)
                        .withGravityType(kGravityType)
                        .withKP(kP)
                        .withKD(kD))
                .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(ARM_GEARBOX_RATIO))
                .withSoftwareLimitSwitch(
                    new SoftwareLimitSwitchConfigs()
                        .withForwardSoftLimitEnable(true)
                        .withForwardSoftLimitThreshold(ARM_SOFT_MAX.in(Rotations))
                        .withReverseSoftLimitEnable(true)
                        .withReverseSoftLimitThreshold(ARM_SOFT_MIN.in(Rotations)))
                .withMotorOutput(
                    new MotorOutputConfigs()
                        .withNeutralMode(NeutralModeValue.Brake)
                        .withInverted(InvertedValue.CounterClockwise_Positive))
                .withCurrentLimits(
                    new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(Amps.of(40))
                        .withStatorCurrentLimitEnable(true)
                        .withSupplyCurrentLimit(Amps.of(70))
                        .withSupplyCurrentLimitEnable(true))
                .withMotionMagic(
                    new MotionMagicConfigs()
                        .withMotionMagicCruiseVelocity(2)
                        .withMotionMagicAcceleration(10)),
            new AngularMechanismConfig(ARM_GEARBOX_RATIO, ARM_LENGTH, ARM_HARD_MIN, ARM_HARD_MAX),
            new AngularSimConfig(true, Degrees.of(10), Degrees.of(0.25), DegreesPerSecond.of(0.25)),
            new SysIdRoutine.Config(
                Volts.of(1).per(Second),
                Volts.of(4),
                Seconds.of(5),
                state -> SignalLogger.writeString("AlgaeArm-sysid-state", state.toString())));
  }

  public static record ElevatorConfig(
      TalonFXID MotorID,
      TalonFXID SecondaryMotorID,
      TalonFXConfiguration MotorConfig,
      LinearMechanismConfig MechanismConfig,
      LinearSimConfig SimConfig,
      SysIdRoutine.Config SysIdConfig) {

    public ElevatorConfig {
      Preferences.initDouble("Elevator/Stow", 2);
      Preferences.initDouble("Elevator/L1", 10);
      Preferences.initDouble("Elevator/L2", 23);
      Preferences.initDouble("Elevator/L3", 38);
      Preferences.initDouble("Elevator/L4", 78);
      Preferences.initDouble("Elevator/Cruise Velocity", 20);
      Preferences.initDouble("Elevator/Acceleration", 35);
      Preferences.initDouble("Elevator/Jerk", 0.0);
      Preferences.initDouble("Elevator/SysId-Buffer", 5);
    }

    public Distance StowHeight() {
      return Inches.of(Preferences.getDouble("Elevator/Stow", 0));
    }

    public Distance L1Height() {
      return Inches.of(Preferences.getDouble("Elevator/L1", 10));
    }

    public Distance L2Height() {
      return Inches.of(Preferences.getDouble("Elevator/L2", 23));
    }

    public Distance L3Height() {
      return Inches.of(Preferences.getDouble("Elevator/L3", 38));
    }

    public Distance L4Height() {
      return Inches.of(Preferences.getDouble("Elevator/L4", 78));
    }

    public AngularVelocity MMCruiseVelocity() {
      return RotationsPerSecond.of(Preferences.getDouble("Elevator/Cruise Velocity", 20));
    }

    public AngularAcceleration MMAcceleration() {
      return RotationsPerSecondPerSecond.of(Preferences.getDouble("Elevator/Acceleration", 35));
    }

    public Velocity<AngularAccelerationUnit> MMJerk() {
      return ANGULAR_JERK.of(Preferences.getDouble("Elevator/Jerk", 0.0));
    }

    public Distance SysIdBuffer() {
      return Inches.of(Preferences.getDouble("Elevator/SysId-Buffer", 5));
    }

    public Distance SysIdMin() {
      return MechanismConfig.MinPosition.plus(SysIdBuffer());
    }

    public Distance SysIdMax() {
      return MechanismConfig.MaxPosition.minus(SysIdBuffer());
    }

    // measured empirically - meters / rotations
    private static final Double ELEVATOR_DRUM_RATIO = 0.15; // Meters per Rotation
    private static final Double ELEVATOR_GEARBOX_RATIO = 42.0 / 10;
    // hard stop
    private static final Distance ELEVATOR_SOFT_MIN = Inches.of(2);
    private static final Distance ELEVATOR_HARD_MIN = Inches.zero();
    // couple inches before upper hard stop
    private static final Distance ELEVATOR_MAX = Meters.of(2);

    private static final Double kS = 0.20996;
    private static final Double kV = 0.57408;
    private static final Double kA = 0.020402;
    private static final Double kG = 0.49136;
    private static final GravityTypeValue kGravityType = GravityTypeValue.Elevator_Static;
    private static final Double kP = 65.059;
    private static final Double kD = 0.0; // 1.5946;

    public static final ElevatorConfig DEFAULT =
        new ElevatorConfig(
            new TalonFXID(11, "3538_Robot"),
            new TalonFXID(12, "3538_Robot"),
            new TalonFXConfiguration()
                .withSlot0(
                    new Slot0Configs()
                        .withKS(kS)
                        .withKV(kV)
                        .withKA(kA)
                        .withKG(kG)
                        .withGravityType(kGravityType)
                        .withKP(kP)
                        .withKD(kD))
                .withSlot1(new Slot1Configs().withKP(200).withKD(20).withKG(22.9).withKS(5.25))
                .withFeedback(
                    new FeedbackConfigs().withSensorToMechanismRatio(ELEVATOR_GEARBOX_RATIO))
                .withSoftwareLimitSwitch(
                    new SoftwareLimitSwitchConfigs()
                        .withForwardSoftLimitEnable(true)
                        .withForwardSoftLimitThreshold(
                            ELEVATOR_MAX.in(Meters) / ELEVATOR_DRUM_RATIO)
                        .withReverseSoftLimitEnable(true)
                        .withReverseSoftLimitThreshold(
                            ELEVATOR_SOFT_MIN.in(Meters) / ELEVATOR_DRUM_RATIO))
                .withMotorOutput(
                    new MotorOutputConfigs()
                        .withNeutralMode(NeutralModeValue.Coast)
                        .withInverted(InvertedValue.CounterClockwise_Positive))
                .withCurrentLimits(
                    new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(Amps.of(120))
                        .withStatorCurrentLimitEnable(true)
                        .withSupplyCurrentLimit(Amps.of(70))
                        .withSupplyCurrentLimitEnable(true))
                .withMotionMagic(
                    new MotionMagicConfigs()
                        .withMotionMagicCruiseVelocity(20)
                        .withMotionMagicAcceleration(93.33333)
                        .withMotionMagicJerk(853.3333)
                        .withMotionMagicExpo_kV(0.56)
                        .withMotionMagicExpo_kA(0.1)),
            new LinearMechanismConfig(
                ELEVATOR_GEARBOX_RATIO, ELEVATOR_DRUM_RATIO, ELEVATOR_HARD_MIN, ELEVATOR_MAX),
            new LinearSimConfig(true, Inches.of(10), Inches.of(0.25), InchesPerSecond.of(0.25)),
            new SysIdRoutine.Config(
                Volts.of(1).per(Second),
                Volts.of(6),
                Seconds.of(15),
                state -> SignalLogger.writeString("Elevator-sysid-state", state.toString())));
  }

  public static record FunnelConfig(
      TalonFXID MotorID,
      TalonFXConfiguration MotorConfig,
      CANrangeID CANrangeID,
      CANrangeConfiguration CANrangeConfig,
      VictorSPXID FunnelSolenoidID,
      Double actuatePower,
      Double standbyPower) {

    public FunnelConfig {
      Preferences.initDouble("Funnel/Intake Power", 0.5);
      Preferences.initDouble("Funnel/Outake Power", -0.5);
    }

    public double IntakePower() {
      return Preferences.getDouble("Funnel/Intake Power", 0.5);
    }

    public double OutakePower() {
      return Preferences.getDouble("Funnel/Outake Power", -0.5);
    }

    public static final FunnelConfig DEFAULT =
        new FunnelConfig(
            new TalonFXID(13, "3538_Robot"),
            new TalonFXConfiguration()
                .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake))
                .withCurrentLimits(
                    new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(Amps.of(40))
                        .withStatorCurrentLimitEnable(true)
                        .withSupplyCurrentLimit(Amps.of(40))
                        .withSupplyCurrentLimitEnable(true)),
            new CANrangeID(21, "3538_Robot"),
            new CANrangeConfiguration()
                .withProximityParams(
                    new ProximityParamsConfigs().withMinSignalStrengthForValidMeasurement(20000)),
            new VictorSPXID(1),
            1.0,
            0.0);
  }

  public static record ClimberConfig(TalonFXID MotorID, TalonFXConfiguration MotorConfig) {
    public ClimberConfig {
      Preferences.initDouble("Climber/Deploy", 18);
    }

    public Angle ClimberDeploy() {
      return Degrees.of((Preferences.getDouble("Climber/Deploy", 18)));
    }

    private static final Double ClimberGearBoxRatio = 24.0 / 1.0;
    private static final Double CLIMBER_MAX_DISTANCE = 1.0;

    public static final ClimberConfig DEFAULT =
        new ClimberConfig(
            new TalonFXID(17, "3538_Robot"),
            new TalonFXConfiguration()
                .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(ClimberGearBoxRatio))
                .withSoftwareLimitSwitch(
                    new SoftwareLimitSwitchConfigs()
                        .withForwardSoftLimitEnable(false)
                        .withForwardSoftLimitThreshold(CLIMBER_MAX_DISTANCE)
                        .withReverseSoftLimitEnable(false)
                        .withReverseSoftLimitThreshold(0))
                .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake))
                .withCurrentLimits(
                    new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(Amps.of(120))
                        .withStatorCurrentLimitEnable(true)
                        .withSupplyCurrentLimit(Amps.of(60))
                        .withSupplyCurrentLimitEnable(true))
                .withSlot0(new Slot0Configs().withKP(30))
                .withMotionMagic(
                    new MotionMagicConfigs()
                        .withMotionMagicCruiseVelocity(4)
                        .withMotionMagicAcceleration(20)));
  }

  public static record TalonFXID(int ID, String CANBus) {
    public TalonFX getMotor() {
      return new TalonFX(ID, CANBus);
    }
  }

  public static record CANdleID(int ID, String CANBus) {
    public CANdle getLED() {
      return new CANdle(ID, CANBus);
    }
  }

  public static record CANdiID(int ID, String CANBus) {
    public CANdi getCANdi() {
      return new CANdi(ID, CANBus);
    }
  }

  public static record CANrangeID(int ID, String CANBus) {
    public CANrange getCANrange() {
      return new CANrange(ID, CANBus);
    }
  }

  public static record VictorSPXID(int ID) {
    public PWMVictorSPX getVictorSPX() {
      return new PWMVictorSPX(ID);
    }
  }

  public static record LinearSimConfig(
      boolean SimulateGravity,
      Distance InitialPosition,
      Distance PositionStdDev,
      LinearVelocity VelocityStdDev) {}

  public static record LinearMechanismConfig(
      Double GearboxRatio, Double DrumRatio, Distance MinPosition, Distance MaxPosition) {}

  public static record AngularSimConfig(
      boolean SimulateGravity,
      Angle InitialPosition,
      Angle PositionStdDev,
      AngularVelocity VelocityStdDev) {}

  public static record AngularMechanismConfig(
      Double GearboxRatio, Distance ArmLength, Angle MinPosition, Angle MaxPosition) {}

  public static record AlgaeMechConfig(TalonFXID MotorID, TalonFXConfiguration MotorConfig) {

    public AlgaeMechConfig {
      Preferences.initDouble("AlgaeMech/Intake Power", 0.5);
      Preferences.initDouble("AlgaeMech/Score Power", 0.5);
      Preferences.initDouble("AlgaeMech/Intake Persist Duration", 0.5);
      Preferences.initDouble("AlgaeMech/Score Persist Duration", 0.5);
    }

    public double IntakePower() {
      return Preferences.getDouble("AlgaeMech/Intake Power", 0.5);
    }

    public double ScorePower() {
      return Preferences.getDouble("AlgaeMech/Score Power", 0.5);
    }

    public double IntakeDelay() {
      return Preferences.getDouble("AlgaeMech/Intake Persist Duration", 0.5);
    }

    public double ScoreDelay() {
      return Preferences.getDouble("AlgaeMech/Score Persist Duration", 0.5);
    }

    public static AlgaeMechConfig DEFAULT =
        new AlgaeMechConfig(
            new TalonFXID(16, "rio"),
            new TalonFXConfiguration()
                .withCurrentLimits(
                    new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(60)
                        .withStatorCurrentLimitEnable(true)));
  }

  public static record CoralMechConfig(
      TalonFXID MotorID,
      TalonFXConfiguration motorconfig,
      CANdiID candiID,
      CANdiConfiguration candiconfig) {

    public CoralMechConfig {
      Preferences.initDouble("CoralMech/Intake Power", 0.5);
      Preferences.initDouble("CoralMech/Intake Persist Duration", 0.5);
      Preferences.initDouble("CoralMech/Intake Pullback Power", 0.5);
      Preferences.initDouble("CoralMech/Intake Pullback Duration", 0.1);
      Preferences.initDouble("CoralMech/Score Power", 0.5);
      Preferences.initDouble("CoralMech/Score Persist Duration", 0.5);

      Preferences.initDouble("CoralMech/L1/Power", 0.5);
      Preferences.initDouble("CoralMech/L1/Duration", 0.5);
      Preferences.initDouble("CoralMech/L2/Power", 0.5);
      Preferences.initDouble("CoralMech/L2/Duration", 0.5);
      Preferences.initDouble("CoralMech/L3/Power", 0.5);
      Preferences.initDouble("CoralMech/L3/Duration", 0.5);
      Preferences.initDouble("CoralMech/L4/Power", 0.5);
      Preferences.initDouble("CoralMech/L4/Duration", 0.5);
    }

    public double IntakeDelay() {
      return Preferences.getDouble("CoralMech/Intake Persist Duration", 0.5);
    }

    public double IntakePower() {
      return Preferences.getDouble("CoralMech/Intake Power", 0.5);
    }

    public double IntakeRetractDuration() {
      return Preferences.getDouble("CoralMech/Intake Pullback Duration", 0.1);
    }

    public double IntakeRetractPower() {
      return Preferences.getDouble("CoralMech/Intake Pullback Power", 0.5);
    }

    public double DefaultScoreDelay() {
      return Preferences.getDouble("CoralMech/Score Persist Duration", 0.5);
    }

    public double DefaultScorePower() {
      return Preferences.getDouble("CoralMech/Score Power", 0.5);
    }

    public double L1ScoreDelay() {
      return Preferences.getDouble("CoralMech/L1/Duration", DefaultScoreDelay());
    }

    public double L1ScorePower() {
      return Preferences.getDouble("CoralMech/L1/Power", DefaultScorePower());
    }

    public double L2ScoreDelay() {
      return Preferences.getDouble("CoralMech/L2/Duration", DefaultScoreDelay());
    }

    public double L2ScorePower() {
      return Preferences.getDouble("CoralMech/L2/Power", DefaultScorePower());
    }

    public double L3ScoreDelay() {
      return Preferences.getDouble("CoralMech/L3/Duration", DefaultScoreDelay());
    }

    public double L3ScorePower() {
      return Preferences.getDouble("CoralMech/L3/Power", DefaultScorePower());
    }

    public double L4ScoreDelay() {
      return Preferences.getDouble("CoralMech/L4/Duration", DefaultScoreDelay());
    }

    public double L4ScorePower() {
      return Preferences.getDouble("CoralMech/L4/Power", DefaultScorePower());
    }

    public static CoralMechConfig DEFAULT =
        new CoralMechConfig(
            new TalonFXID(14, "rio"),
            new TalonFXConfiguration(),
            new CANdiID(20, "rio"),
            new CANdiConfiguration());
  }

  public static record LEDConfig(
      CANdleID CANdleID,
      CANdleConfiguration CANdleConfig,
      Color CycleModeColor,
      Color ClimbModeColor,
      Color DefenseModeColor) {
    public LEDConfig {}

    public static final LEDConfig DEFAULT =
        new LEDConfig(
            new CANdleID(5, "3538_Robot"),
            new CANdleConfiguration(),
            Color.kYellow,
            Color.kTeal,
            Color.kPurple);
  }
}
