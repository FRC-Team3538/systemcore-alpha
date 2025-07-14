// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.lib;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.constants.FieldGeometry;

public class AllianceFlipUtil {

  public static double applyX(double x) {
    return applyX(x, shouldFlip());
  }

  public static double applyX(double x, boolean force) {
    return force ? FieldGeometry.fieldLength - x : x;
  }

  public static double applyY(double y) {
    return applyY(y, shouldFlip());
  }

  public static double applyY(double y, boolean force) {
    return force ? FieldGeometry.fieldWidth - y : y;
  }

  public static Translation2d rotate(Translation2d translation) {
    return new Translation2d(applyX(translation.getX()), applyY(translation.getY()));
  }

  public static Rotation2d rotate(Rotation2d rotation) {
    return shouldFlip() ? rotation.rotateBy(Rotation2d.kPi) : rotation;
  }

  public static Pose2d rotate(Pose2d pose) {
    return shouldFlip()
        ? new Pose2d(rotate(pose.getTranslation()), rotate(pose.getRotation()))
        : pose;
  }

  public static Translation3d rotate(Translation3d translation) {
    return new Translation3d(
        applyX(translation.getX()), applyY(translation.getY()), translation.getZ());
  }

  public static Rotation3d rotate(Rotation3d rotation) {
    return shouldFlip() ? rotation.rotateBy(new Rotation3d(0.0, 0.0, Math.PI)) : rotation;
  }

  public static Pose3d rotate(Pose3d pose) {
    return new Pose3d(rotate(pose.getTranslation()), rotate(pose.getRotation()));
  }

  public static Translation2d mirror(Translation2d translation) {
    return mirror(translation, shouldFlip());
  }

  public static Translation2d mirror(Translation2d translation, boolean force) {
    return new Translation2d(applyX(translation.getX(), force), translation.getY());
  }

  public static Rotation2d mirror(Rotation2d rotation) {
    return mirror(rotation, shouldFlip());
  }

  public static Rotation2d mirror(Rotation2d rotation, boolean force) {
    return shouldFlip() ? Rotation2d.k180deg.minus(rotation) : rotation;
  }

  public static Pose2d mirror(Pose2d pose) {
    return mirror(pose, shouldFlip());
  }

  public static Pose2d mirror(Pose2d pose, boolean force) {
    return shouldFlip()
        ? new Pose2d(mirror(pose.getTranslation(), force), mirror(pose.getRotation(), force))
        : pose;
  }

  public static Translation3d mirror(Translation3d translation) {
    return new Translation3d(applyX(translation.getX()), translation.getY(), translation.getZ());
  }

  public static Rotation3d mirror(Rotation3d rotation) {
    return shouldFlip() ? new Rotation3d(0.0, 0.0, Math.PI).minus(rotation) : rotation;
  }

  public static Pose3d mirror(Pose3d pose) {
    return new Pose3d(mirror(pose.getTranslation()), mirror(pose.getRotation()));
  }

  public static boolean shouldFlip() {
    return DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
  }
}
