package frc.robot.auto;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;
import static frc.robot.auto.Setpoint.*;
import static frc.robot.oi.StructureState.INTAKE_ALGAE_L2;
import static frc.robot.oi.StructureState.INTAKE_ALGAE_L3;
import static frc.robot.oi.StructureState.PRIME_INTAKE_ALGAE_L2;
import static frc.robot.oi.StructureState.PRIME_INTAKE_ALGAE_L3;
import static frc.robot.oi.StructureState.SCORE_ALGAE_BARGE;
import static frc.robot.oi.StructureState.SCORE_CORAL_L4;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.oi.StructureState;
import frc.robot.subsystems.AlgaeMech;
import frc.robot.subsystems.CoralMech;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Funnel;
import frc.robot.subsystems.Superstructure;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.function.Function;

public class Autos {
  private final AutoFactory autoFactory;

  private final Drive drive;
  private final Superstructure superstructure;
  private final CoralMech coralMech;
  private final AlgaeMech algaeMech;
  private final Funnel funnel;
  private final Elevator elevator;
  private final Function<StructureState, Command> innerAutoScore;

  public Autos(
      Drive drive,
      Superstructure superstructure,
      CoralMech coralMech,
      AlgaeMech algaeMech,
      Funnel funnel,
      Elevator elevator,
      Function<StructureState, Command> AutoScore) {
    autoFactory =
        new AutoFactory(
            () -> drive.getStateCopy().Pose,
            drive::resetPose,
            drive::followPath,
            true,
            drive,
            (trajectory, starting) -> {
              // RJLog.log("Choreo/Active Trajectory/Name", starting ? trajectory.name() : "");
              // if (DriverStation.getAlliance()
              //     .filter(alliance -> alliance == Alliance.Red)
              //     .isPresent()) {
              //   RJLog.log("Choreo/Active Trajectory", trajectory.flipped());
              // } else {
              //   RJLog.log("Choreo/Active Trajectory", trajectory);
              // }
              // RJLog.log(String.format("Choreo/Trajectory/%s", trajectory.name()), trajectory);
            });

    this.drive = drive;
    this.superstructure = superstructure;
    this.coralMech = coralMech;
    this.algaeMech = algaeMech;
    this.funnel = funnel;
    this.elevator = elevator;
    this.innerAutoScore = AutoScore;
  }

  public void configureAutoChooser(AutoChooser autoChooser) {
    autoChooser.addRoutine("4x L4 Coral Left", this::LeftSide4xL4);
    autoChooser.addRoutine("4x L4 Coral Right", this::RightSide4xL4);
    autoChooser.addRoutine("4x L4 Coral Left - COMPAT", this::LeftSide4xL4_COMPAT);
    autoChooser.addRoutine("4x L4 Coral Right - COMPAT", this::RightSide4xL4_COMPAT);

    // autoChooser.addRoutine("4x L4 Coral Right - PATHS", this::RightSide4xL4Paths);
    // autoChooser.addRoutine("4x L4 Coral Left - PATHS", this::LeftSide4xL4Paths);
    // autoChooser.addRoutine("4x L4 Coral Right - PATHS - COMPAT",
    // this::RightSide4xL4Paths_COMPAT);
    // autoChooser.addRoutine("4x L4 Coral Left - PATHS - COMPAT", this::LeftSide4xL4Paths_COMPAT);
    autoChooser.addRoutine("1x L4 Coral 3x Algae Barge", this::CenterAlgae);
    autoChooser.addRoutine("1x L4 Coral 3x Algae Barge - PATHS", this::CenterAlgaePaths);
  }

  public AutoRoutine LeftSide4xL4() {
    return ScoreL4Sequence("Auton/LeftSide4xL4", SL, J, K, L, A);
  }

  public AutoRoutine RightSide4xL4() {
    return ScoreL4Sequence("Auton/RightSide4xL4", SR, E, D, C, B);
  }

  public AutoRoutine LeftSide4xL4_COMPAT() {
    return ScoreL4Sequence("Auton/LeftSide4xL4", SL, J, K, L, I);
  }

  public AutoRoutine RightSide4xL4_COMPAT() {
    return ScoreL4Sequence("Auton/RightSide4xL4", SR, E, D, C, F);
  }

  public AutoRoutine LeftSide4xL4Paths() {
    return ScoreL4SequencePaths("Auton/LeftSide4xL4 - PATHS", SL, J, K, L, A);
  }

  public AutoRoutine RightSide4xL4Paths() {
    return ScoreL4SequencePaths("Auton/RightSide4xL4 - PATHS", SR, E, D, C, B);
  }

  public AutoRoutine LeftSide4xL4Paths_COMPAT() {
    return ScoreL4SequencePaths("Auton/LeftSide4xL4 - PATHS", SL, J, K, L, I);
  }

  public AutoRoutine RightSide4xL4Paths_COMPAT() {
    return ScoreL4SequencePaths("Auton/RightSide4xL4 - PATHS", SR, E, D, C, F);
  }

  public AutoRoutine CenterAlgaePaths() {
    var routine = autoFactory.newRoutine("Auton/CenterAlgae - PATHS");

    var StartToG = routine.trajectory("Barge Auto", 0);
    var GToBarge = routine.trajectory("Barge Auto", 1);
    var BargeToE = routine.trajectory("Barge Auto", 2);
    var EToBarge = routine.trajectory("Barge Auto", 3);
    var BargeToI = routine.trajectory("Barge Auto", 4);
    var IToBarge = routine.trajectory("Barge Auto", 5);
    var BargeToCoralStation = routine.trajectory("Barge Auto", 6);

    var cmd = StartToG.cmd();

    routine.active().onTrue(StartToG.resetOdometry().andThen(cmd).withName(cmd.getName()));

    StartToG.done().onTrue(drive.HoldState(StartToG.getFinalPose().get(), new ChassisSpeeds()));
    StartToG.doneDelayed(0.5).onTrue(GToBarge.cmd());

    GToBarge.done().onTrue(drive.HoldState(GToBarge.getFinalPose().get(), new ChassisSpeeds()));
    GToBarge.doneDelayed(0.5).onTrue(BargeToE.cmd());

    BargeToE.done().onTrue(drive.HoldState(BargeToE.getFinalPose().get(), new ChassisSpeeds()));
    BargeToE.doneDelayed(0.5).onTrue(EToBarge.cmd());

    EToBarge.done().onTrue(drive.HoldState(EToBarge.getFinalPose().get(), new ChassisSpeeds()));
    EToBarge.doneDelayed(0.5).onTrue(BargeToI.cmd());

    BargeToI.done().onTrue(drive.HoldState(BargeToI.getFinalPose().get(), new ChassisSpeeds()));
    BargeToI.doneDelayed(0.5).onTrue(IToBarge.cmd());

    IToBarge.done().onTrue(drive.HoldState(IToBarge.getFinalPose().get(), new ChassisSpeeds()));
    IToBarge.doneDelayed(0.5).onTrue(BargeToCoralStation.cmd());

    return routine;
  }

  public AutoRoutine CenterAlgae() {
    var routine = autoFactory.newRoutine("Auton/CenterAlgae");

    var StartToG = routine.trajectory("Barge Auto", 0);
    var GToBarge = routine.trajectory("Barge Auto", 1);
    var BargeToE = routine.trajectory("Barge Auto", 2);
    var EToBarge = routine.trajectory("Barge Auto", 3);
    var BargeToI = routine.trajectory("Barge Auto", 4);
    var IToBarge = routine.trajectory("Barge Auto", 5);
    var BargeToCoralStation = routine.trajectory("Barge Auto", 6);

    var cmd = StartToG.cmd();

    routine.active().onTrue(StartToG.resetOdometry().andThen(cmd).withName(cmd.getName()));

    var coralInMech = routine.observe(coralMech::hasCoral);
    var algaeInMech = routine.observe(algaeMech::HoldsAlgae);

    AlgaePickupSequence(
        StartToG,
        PRIME_INTAKE_ALGAE_L2,
        INTAKE_ALGAE_L2,
        Optional.of(SCORE_CORAL_L4),
        coralInMech,
        algaeInMech,
        GToBarge);
    ScoreAlgaeSequence(GToBarge, algaeInMech, BargeToE);

    AlgaePickupSequence(
        BargeToE,
        PRIME_INTAKE_ALGAE_L3,
        INTAKE_ALGAE_L3,
        Optional.empty(),
        coralInMech,
        algaeInMech,
        EToBarge);
    ScoreAlgaeSequence(EToBarge, algaeInMech, BargeToI);

    AlgaePickupSequence(
        BargeToI,
        PRIME_INTAKE_ALGAE_L3,
        INTAKE_ALGAE_L3,
        Optional.empty(),
        coralInMech,
        algaeInMech,
        IToBarge);
    ScoreAlgaeSequence(IToBarge, algaeInMech, BargeToCoralStation);

    BargeToCoralStation.active().onTrue(superstructure.Home());

    return routine;
  }

  private void AlgaePickupSequence(
      AutoTrajectory trajectory,
      StructureState prePickupState,
      StructureState pickupHeight,
      Optional<StructureState> scoreCoralHeight,
      Trigger coralInMech,
      Trigger algaeInMech,
      AutoTrajectory followup) {
    trajectory.done().onTrue(drive.HoldState(trajectory.getFinalPose().get(), new ChassisSpeeds()));

    if (scoreCoralHeight.isPresent()) {
      trajectory
          .atTimeBeforeEnd(0.6)
          .onTrue(AutoScore(trajectory.getFinalPose(), scoreCoralHeight.get(), coralInMech));
      trajectory.recentlyDone().and(coralInMech.negate()).onTrue(PickAlgae(pickupHeight));
      trajectory.recentlyDone().and(algaeInMech).onTrue(followup.cmd());
    } else {
      trajectory.active().onTrue(superstructure.SetpointCommand(prePickupState));
      trajectory.atTimeBeforeEnd(0.3).onTrue(PickAlgae(pickupHeight));
      trajectory.recentlyDone().and(algaeInMech).onTrue(Commands.wait(0.3).andThen(followup.cmd()));
    }
  }

  private void ScoreAlgaeSequence(
      AutoTrajectory trajectory, Trigger algaeInMech, AutoTrajectory followup) {
    trajectory.active().onTrue(superstructure.PrepareAlgaeL3());
    trajectory.done().onTrue(drive.HoldState(trajectory.getFinalPose().get(), new ChassisSpeeds()));
    trajectory
        .atTimeBeforeEnd(0.8)
        .onTrue(ScoreAlgae(trajectory.getFinalPose(), SCORE_ALGAE_BARGE));
    trajectory.recentlyDone().and(algaeInMech.negate()).onTrue(followup.cmd());
  }

  private AutoRoutine ScoreL4Sequence(String name, Setpoint pickupLocation, Setpoint... branches) {
    var routine = autoFactory.newRoutine(name);

    if (branches.length == 0) {
      return routine;
    }

    var state = new AtomicInteger(0);

    var coralInFunnel = routine.observe(funnel.CoralInFeeder());
    var coralInMech = routine.observe(() -> coralMech.hasCoral());

    routine.active().onTrue(runOnce(() -> state.set(0)).withName("Auton::Start"));

    for (var step = 0; step < branches.length; step++) {
      final var thisStep = step;

      Trigger startCondition;
      AutoTrajectory approach;
      AutoTrajectory retreat = routine.trajectory(branches[step].to(pickupLocation));

      if (step == 0) {
        startCondition = routine.active();
        approach = routine.trajectory(START.to(branches[step]));
      } else {
        startCondition = coralInFunnel;
        approach = routine.trajectory(pickupLocation.to(branches[step]));
      }

      ApproachAndScore(
          step == 0,
          startCondition.and(() -> state.get() == thisStep),
          coralInMech.and(() -> state.get() == thisStep),
          coralInFunnel.and(() -> state.get() == thisStep),
          approach,
          SCORE_CORAL_L4);
      RetreatAndPickupCoral(approach.doneFor(3).and(() -> !coralMech.hasCoral()), retreat, state);
    }

    return routine;
  }

  private final double scoreWaitTime = 0.5; // seconds
  private final double intakeWaitTime = 0.5; // seconds

  public AutoRoutine ScoreL4SequencePaths(
      String name, Setpoint pickupLocation, Setpoint... branches) {
    var routine = autoFactory.newRoutine(name);

    if (branches.length == 0) {
      return routine;
    }

    var firstPath = routine.trajectory(START.to(branches[0]));

    routine.active().onTrue(firstPath.resetOdometry().withName("Auton::Start"));

    AutoTrajectory lastPath = null;

    for (var step = 0; step < branches.length; step++) {
      AutoTrajectory approach;
      if (step == 0) {
        approach = routine.trajectory(START.to(branches[step]));

        routine.active().onTrue(approach.cmd());
      } else {
        approach = routine.trajectory(pickupLocation.to(branches[step]));

        lastPath.doneDelayed(intakeWaitTime).onTrue(approach.cmd());
      }

      approach
          .doneFor(0.1)
          .onTrue(drive.HoldState(approach.getFinalPose().get(), new ChassisSpeeds()));

      AutoTrajectory retreat = routine.trajectory(branches[step].to(pickupLocation));

      approach.doneDelayed(scoreWaitTime).onTrue(retreat.cmd());

      retreat
          .doneFor(0.1)
          .onTrue(drive.HoldState(retreat.getFinalPose().get(), new ChassisSpeeds()));

      lastPath = retreat;
    }

    return routine;
  }

  private void ApproachAndScore(
      boolean firstPath,
      Trigger start,
      Trigger coralInMech,
      Trigger coralInFunnel,
      AutoTrajectory approach,
      StructureState scoreConfiguration) {

    if (firstPath) {
      var approachCmd = approach.cmd();
      start.onTrue(approach.resetOdometry().andThen(approachCmd).withName(approachCmd.getName()));
    } else {
      coralInMech
          .and(approach.inactive())
          .and(approach.recentlyDone().negate())
          .onTrue(approach.cmd());
      start.onTrue(waitUntil(coralInFunnel).andThen(approach.spawnCmd()));
    }

    approach
        .doneFor(0.1)
        .onTrue(drive.HoldState(approach.getFinalPose().get(), new ChassisSpeeds()));

    approach
        .atTimeBeforeEnd(0.9)
        .onTrue(AutoScore(approach.getFinalPose(), scoreConfiguration, coralInMech));
  }

  private Command PickAlgae(StructureState state) {
    return superstructure.SetpointCommand(state)
        .alongWith(algaeMech.Intake())
        .withName("PickAlgae");
  }

  private Command ScoreAlgae(Optional<Pose2d> target, StructureState state) {
    return superstructure.SetpointCommand(state)
        .until(
            () -> {
              var driveInPosition = target.map(drive::IsAtPose).orElse(false);
              var superstructureInPosition = superstructure.WithinTolerance(state);

              return driveInPosition && superstructureInPosition;
            })
        .withTimeout(1.5)
        .andThen(algaeMech.AutonScore())
        .withName("ScoreAlgae");
  }

  private void RetreatAndPickupCoral(Trigger start, AutoTrajectory retreat, AtomicInteger state) {
    start.onTrue(retreat.cmd());

    retreat.active().onTrue(runOnce(state::incrementAndGet).withName("Auton::IncrementState"));

    retreat.atTime(0.25).onTrue(superstructure.IntakeCoral());

    retreat.doneFor(0.1).onTrue(drive.HoldState(retreat.getFinalPose().get(), new ChassisSpeeds()));

    retreat
        .atTimeBeforeEnd(1.0)
        .onTrue(coralMech.Intake().deadlineFor(funnel.Intake()).withName("LoadCoral"));
  }

  private Command AutoScore(Optional<Pose2d> target, StructureState state, Trigger coralInMech) {
    return sequence(
            waitUntil(coralInMech),
            superstructure.SetpointCommand(state)
                .until(
                    () -> {
                      var driveInPosition = target.map(drive::IsAtPose).orElse(false);
                      var superstructureInPosition = superstructure.WithinTolerance(state);

                      return driveInPosition && superstructureInPosition;
                    })
                .withTimeout(1.5),
            innerAutoScore.apply(state))
        .withName("AutoScore");
  }
}
