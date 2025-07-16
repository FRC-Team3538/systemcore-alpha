// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.vision;

import static frc.robot.constants.VisionConstants.*;
import static java.lang.Math.max;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.VisionConstants;
import frc.robot.lib.RJLog;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import frc.robot.subsystems.vision.VisionIO.VisionIOInputs;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;

public class Vision extends SubsystemBase {
  private final VisionConsumer consumer;
  private final VisionIO[] io;
  private final VisionIOInputs[] inputs;
  private final Alert[] disconnectedAlerts;

  private BooleanTopic pvSettingTopic;
  private BooleanPublisher pvSettingPublisher;

  private boolean enabled = true;

  private Set<String> disabledCameras = new HashSet<>();

  public Vision(VisionConsumer consumer, VisionIO... io) {
    this.consumer = consumer;
    this.io = io;

    // Initialize inputs
    this.inputs = new VisionIOInputs[io.length];
    for (int i = 0; i < inputs.length; i++) {
      inputs[i] = new VisionIOInputs();
    }

    // Initialize disconnected alerts
    this.disconnectedAlerts = new Alert[io.length];
    for (int i = 0; i < inputs.length; i++) {
      disconnectedAlerts[i] =
          new Alert("Vision camera " + io[i].getName() + " is disconnected.", AlertType.kWarning);
    }

    Preferences.initBoolean("Photonvision/UseNewFrametime", false);
    pvSettingTopic =
        NetworkTableInstance.getDefault().getBooleanTopic("/photonvision/use_new_cscore_frametime");
    pvSettingPublisher = pvSettingTopic.publish();

    setDefaultCommand(enableCommand());
  }

  public boolean enabled() {
    return enabled;
  }

  public Command enableCommand() {
    return run(this::enable).ignoringDisable(true).withName("Vision::Enabled");
  }

  public Command disableCommand() {
    return run(this::disable).ignoringDisable(true).withName("Vision::Disabled");
  }

  public void enable() {
    this.enabled = true;
  }

  public void disable() {
    this.enabled = false;
  }

  public Command DisableFrontCamerasCommand() {
    return Commands.runEnd(this::disableFrontCameras, this::enableFrontCameras);
  }

  public Command DisableRearCamerasCommand() {
    return Commands.runEnd(this::disableRearCameras, this::enableRearCameras);
  }

  public void enableFrontCameras() {
    enable(VisionConstants.camera0Name);
    enable(VisionConstants.camera1Name);
  }

  public void disableFrontCameras() {
    disable(VisionConstants.camera0Name);
    disable(VisionConstants.camera1Name);
  }

  public void enableRearCameras() {
    enable(VisionConstants.camera2Name);
    enable(VisionConstants.camera3Name);
  }

  public void disableRearCameras() {
    disable(VisionConstants.camera2Name);
    disable(VisionConstants.camera3Name);
  }

  public void enable(String name) {
    disabledCameras.remove(name);
  }

  public void disable(String name) {
    disabledCameras.add(name);
  }

  /**
   * Returns the X angle to the best target, which can be used for simple servoing with vision.
   *
   * @param cameraIndex The index of the camera to use.
   */
  public Rotation2d getTargetX(int cameraIndex) {
    return inputs[cameraIndex].latestTargetObservation.tx();
  }

  @Override
  public void periodic() {
    RJLog.log("Vision/enabled", enabled);
    SmartDashboard.putBoolean("Vision/enabled", enabled);
    pvSettingPublisher.accept(Preferences.getBoolean("Photonvision/UseNewFrametime", false));

    // Tracer.startTrace("updateInputs");
    for (int i = 0; i < io.length; i++) {
      // Tracer.startTrace(io[i].getName());
      io[i].updateInputs(inputs[i]);
      RJLog.log(String.format("Vision/Camera-%s/Inputs", inputs[i].name), inputs[i]);
      // Tracer.endTrace();
    }
    // Tracer.endTrace();

    // Initialize logging values
    List<Pose3d> allTagPoses = new LinkedList<>();
    List<Pose3d> allRobotPoses = new LinkedList<>();
    List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
    List<Pose3d> allRobotPosesRejected = new LinkedList<>();

    // Loop over cameras
    // Tracer.startTrace("filterInputs");
    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      // Tracer.startTrace(io[cameraIndex].getName());

      // Update disconnected alert
      disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

      // Initialize logging values
      List<Pose3d> tagPoses = new LinkedList<>();
      List<Pose3d> robotPoses = new LinkedList<>();
      List<Pose3d> robotPosesAccepted = new LinkedList<>();
      List<Pose3d> robotPosesRejected = new LinkedList<>();

      // Add tag poses
      // Tracer.startTrace("Tags");
      for (int tagId : inputs[cameraIndex].tagIds) {
        var tagPose = aprilTagLayout.getTagPose(tagId);
        if (tagPose.isPresent()) {
          tagPoses.add(tagPose.get());
        }
      }
      // Tracer.endTrace();

      // Loop over pose observations
      // Tracer.startTrace("ApplyObservations");
      for (int index = 0;
          index < inputs[cameraIndex].poseObservations.length;
          index++) {
        // Tracer.startTrace(Integer.toString(index));

        var observation = inputs[cameraIndex].poseObservations[index];

        // Check whether to reject pose
        boolean rejectPose =
            observation.tagCount() == 0 // Must have at least one tag
                || (observation.tagCount() == 1
                    && observation.ambiguity() > maxAmbiguity) // Cannot be high ambiguity
                || Math.abs(observation.pose().getZ())
                    > maxZError // Must have realistic Z coordinate

                // Must be within the field boundaries
                || observation.pose().getX() < 0.0
                || observation.pose().getX() > aprilTagLayout.getFieldLength()
                || observation.pose().getY() < 0.0
                || observation.pose().getY() > aprilTagLayout.getFieldWidth()
                || disabledCameras.contains(inputs[cameraIndex].name)
                || (observation.averageTagDistance() > 4.06
                    && observation.tagCount() == 1); // Meters; 160 in

        // Add pose to log
        robotPoses.add(observation.pose());
        if (rejectPose) {
          robotPosesRejected.add(observation.pose());
        } else {
          robotPosesAccepted.add(observation.pose());
        }

        // Skip if rejected
        if (rejectPose) {
          // Tracer.endTrace();
          continue;
        }

        // Calculate standard deviations
        double stdDevFactor =
            Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
        double linearStdDev = linearStdDevBaseline * stdDevFactor;
        double angularStdDev = angularStdDevBaseline * stdDevFactor;
        if (observation.type() == PoseObservationType.MEGATAG_2) {
          linearStdDev *= linearStdDevMegatag2Factor;
          angularStdDev *= angularStdDevMegatag2Factor;
        }
        if (cameraIndex < cameraStdDevFactors.length) {
          linearStdDev *= cameraStdDevFactors[cameraIndex];
          angularStdDev *= cameraStdDevFactors[cameraIndex];
        }

        if (!enabled) {
          // Tracer.endTrace();
          continue;
        }

        // Send vision observation
        consumer.accept(
            observation.pose().toPose2d(),
            observation.timestamp(),
            VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
        // Tracer.endTrace();
      }
      // Tracer.endTrace();

      // Log camera datadata
      // Tracer.startTrace("Logging");
      RJLog.log(
          String.format("Vision/Camera-%s/TagPoses", cameraIndex),
          tagPoses.toArray(new Pose3d[tagPoses.size()]));
      RJLog.log(
          String.format("Vision/Camera-%s/RobotPoses", cameraIndex),
          robotPoses.toArray(new Pose3d[robotPoses.size()]));
      RJLog.log(
          String.format("Vision/Camera-%s/RobotPosesAccepted", cameraIndex),
          robotPosesAccepted.toArray(new Pose3d[robotPosesAccepted.size()]));
      RJLog.log(
          String.format("Vision/Camera-%s/RobotPosesRejected", cameraIndex),
          robotPosesRejected.toArray(new Pose3d[robotPosesRejected.size()]));
      // Tracer.endTrace();

      allTagPoses.addAll(tagPoses);
      allRobotPoses.addAll(robotPoses);
      allRobotPosesAccepted.addAll(robotPosesAccepted);
      allRobotPosesRejected.addAll(robotPosesRejected);
      // Tracer.endTrace();
    }
    // Tracer.endTrace();

    // Log summary data
    RJLog.log("Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[allTagPoses.size()]));
    RJLog.log("Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[allRobotPoses.size()]));
    RJLog.log(
        "Vision/Summary/RobotPosesAccepted",
        allRobotPosesAccepted.toArray(new Pose3d[allRobotPosesAccepted.size()]));
    RJLog.log(
        "Vision/Summary/RobotPosesRejected",
        allRobotPosesRejected.toArray(new Pose3d[allRobotPosesRejected.size()]));
  }

  @FunctionalInterface
  public static interface VisionConsumer {
    public void accept(
        Pose2d visionRobotPoseMeters, double timestampSeconds, Vector<N3> visionMeasurementStdDevs);
  }
}
