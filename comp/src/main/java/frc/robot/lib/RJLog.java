package frc.robot.lib;

import choreo.trajectory.Trajectory;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveControlParameters;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import dev.doglog.DogLog;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.vision.VisionIO.PoseObservation;
import frc.robot.subsystems.vision.VisionIO.TargetObservation;
import frc.robot.subsystems.vision.VisionIO.VisionIOInputs;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;

public class RJLog extends DogLog {

  public static void log(String key, Trajectory<?> trajectory) {
    RJLog.log(String.format("%s/totalTime", key), trajectory.getTotalTime());
    RJLog.log(String.format("%s/path", key), trajectory.getPoses());
  }

  public static void log(String key, List<Pose2d> list) {
    log(key, list.toArray(new Pose2d[list.size()]));
  }

  public static void log(String key, ControlRequest request) {
    for (var param : request.getControlInfo().entrySet()) {
      log(String.format("%s/%s/%s", key, request.getName(), param.getKey()), param.getValue());
    }
  }

  public static void log(String key, VisionIOInputs visionInputs) {
    log(String.format("%s/connected", key), visionInputs.connected);
    log(String.format("%s/tagIds", key), visionInputs.tagIds);
    log(String.format("%s/latestTargetObservation", key), visionInputs.latestTargetObservation);
    log(String.format("%s/poseObservations", key), visionInputs.poseObservations);
  }

  public static void log(String key, TargetObservation observation) {
    log(String.format("%s/tx", key), observation.tx());
    log(String.format("%s/ty", key), observation.ty());
  }

  public static void log(String key, PoseObservation observation) {
    log(String.format("%s/timestamp", key), observation.timestamp());
    log(String.format("%s/pose", key), observation.pose());
    log(String.format("%s/ambiguity", key), observation.ambiguity());
    log(String.format("%s/averageTagDistance", key), observation.averageTagDistance());
    log(String.format("%s/tagCount", key), observation.tagCount());
    log(String.format("%s/type", key), observation.type());
  }

  public static void log(String key, PoseObservation[] observations) {
    for (int i = 0; i < observations.length; i++) {
      log(String.format("%s/%s", key, i), observations[i]);
    }
  }

  public static void log(String key, Measure<?> measure) {
    log(String.format("%s/%s", key, measure.baseUnit().name()), measure.baseUnitMagnitude());
  }

  public static void log(String key, Vector<?> vector) {
    log(key, vector.getData());
  }

  public static void log(String key, Matrix<?, ?> vector) {
    log(key, vector.getData());
  }

  public static void log(String key, SwerveDriveState state) {
    log(String.format("%s/Pose", key), state.Pose);
    log(String.format("%s/Speeds", key), state.Speeds);
    log(String.format("%s/ModuleStates", key), state.ModuleStates);
    log(String.format("%s/ModuleTargets", key), state.ModuleTargets);
    log(String.format("%s/ModulePositions", key), state.ModulePositions);
    log(String.format("%s/RawHeading", key), state.RawHeading);
    log(String.format("%s/Timestamp", key), state.Timestamp);
    log(String.format("%s/OdometryPeriod", key), state.OdometryPeriod);
    log(String.format("%s/SuccessfulDaqs", key), state.SuccessfulDaqs);
    log(String.format("%s/FailedDaqs", key), state.FailedDaqs);
  }

  public static void log(String key, SwerveControlParameters requestParameters) {
    log(String.format("%s/Speed", key), requestParameters.currentChassisSpeed);
    log(String.format("%s/Pose", key), requestParameters.currentPose);
    log(String.format("%s/Timestamp", key), requestParameters.timestamp);
    log(String.format("%s/OperatorForward", key), requestParameters.operatorForwardDirection);
    log(String.format("%s/UpdatePeriod", key), requestParameters.updatePeriod);
  }

  public static void logInitializedCommand(Command command) {
    logCommand(command, true, Optional.empty());
  }

  public static void logFinishedCommand(Command command) {
    logCommand(command, false, Optional.empty());
  }

  public static void logInterruptedCommand(Command command, Optional<Command> interruptor) {
    logCommand(command, false, interruptor);
  }

  private static Map<String, Integer> commandCounts = new HashMap<>();

  private static void logCommand(
      Command command, Boolean active, Optional<Command> maybeInterrupt) {
    String name = command.getName();
    int count = commandCounts.getOrDefault(name, 0) + (active ? 1 : -1);
    commandCounts.put(name, count);
    RJLog.log("CommandsUnique/" + name + "_" + Integer.toHexString(command.hashCode()), active);
    RJLog.log("CommandsAll/" + name, count > 0);

    if (maybeInterrupt.isPresent()) {
      Command interruptor = maybeInterrupt.get();
      String interruptorName = interruptor.getName();
      RJLog.log("CommandsAll/" + name + "/interruptor", interruptorName);
    }

    if (active) {
      for (var requirement : command.getRequirements()) {
        RJLog.log(
            String.format("Subsystems/%s/CurrentCommand", requirement.getName()),
            command.getName());
      }
    } else if (maybeInterrupt.isEmpty()) {
      for (var requirement : command.getRequirements()) {
        RJLog.log(String.format("Subsystems/%s/CurrentCommand", requirement.getName()), "");
      }
    }
  }
}
