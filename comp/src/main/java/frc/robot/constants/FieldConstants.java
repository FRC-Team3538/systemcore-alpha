package frc.robot.constants;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Preferences;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

public class FieldConstants {
  private static FieldConstants _INSTANCE;

  public static FieldConstants getInstance() {
    if (_INSTANCE == null) {
      _INSTANCE = new FieldConstants();
    }

    return _INSTANCE;
  }

  public static enum Side {
    LEFT,
    RIGHT
  }

  private final String LAYOUT_ALERT_FMT = "Failed to load Field preference - falling back to %s";

  private final Alert LAYOUT_ALERT =
      new Alert(
          "Failed to load Field preference - falling back to k2025Reefscape", AlertType.kWarning);

  private Transform2d TagToLeftBranch =
      new Transform2d(Meters.of(-0.30479999999999996), Meters.of(-0.1651), Rotation2d.kZero);

  private Transform2d TagToRightBranch =
      new Transform2d(Meters.of(-0.30479999999999996), Meters.of(0.1651), Rotation2d.kZero);

  private Transform2d BranchToRobot =
      new Transform2d(Meters.of(0.8000999999999999), Meters.zero(), Rotation2d.fromDegrees(180.0));

  private Transform2d CageToRobot =
      new Transform2d(Meters.of(-1.016), Meters.zero(), Rotation2d.fromDegrees(90.0));

  private Transform2d BargeToRobot =
      new Transform2d(Meters.of(-1.05), Meters.zero(), Rotation2d.kZero);

  private Pose2d MirrorSideToSide(Pose2d toMirror) {
    return new Pose2d(
        toMirror.getX(),
        FieldGeometry.fieldWidth - toMirror.getY(),
        toMirror.getRotation().unaryMinus());
  }

  public Pose2d UndefendedIntakePoseRight =
      // 58, 32.5
      new Pose2d(Inches.of(59), Inches.of(34), Rotation2d.fromDegrees(35));
  public Pose2d UndefendedIntakePoseLeft = MirrorSideToSide(UndefendedIntakePoseRight);

  public Pose2d DefendedIntakeForwardRight =
      new Pose2d(Inches.of(73), Inches.of(22.1), Rotation2d.fromDegrees(30));
  public Pose2d DefendedIntakeBackRight =
      new Pose2d(Inches.of(20.5), Inches.of(60), Rotation2d.fromDegrees(75));
  public Pose2d DefendedIntakeForwardLeft = MirrorSideToSide(DefendedIntakeForwardRight);
  public Pose2d DefendedIntakeBackLeft = MirrorSideToSide(DefendedIntakeBackRight);

  public final AprilTagFieldLayout layout = loadField();

  public final List<Pair<Integer, Pose2d>> blueReefTags = new ArrayList<>();
  public final List<Pair<Integer, Pose2d>> redReefTags = new ArrayList<>();
  public final List<Pair<Integer, Pose2d>> allReefTags = new ArrayList<>();

  private final int[] algaeLevelAboveTag =
      new int[] {
        0,
        0, // Red Coral Stations
        0,
        0,
        0, // Red Processor & Barge
        2,
        3,
        2,
        3,
        2,
        3, // Red Reef
        0,
        0, // Blue Coral Stations
        0,
        0,
        0, // Blue Processor & Barge
        2,
        3,
        2,
        3,
        2,
        3 // Blue Reef
      };

  public int AlgaeHeightOnClosestReefSide(Pose2d pose) {
    var tag = getClosestReefTag(pose);

    return algaeLevelAboveTag[tag - 1];
  }

  public FieldConstants() {
    Preferences.initString("Field/Path", "");
    Preferences.initString("Field/Layout", "k2025ReefscapeWelded");

    blueReefTags.add(Pair.of(18, layout.getTagPose(18).get().toPose2d()));

    blueReefTags.add(Pair.of(17, layout.getTagPose(17).get().toPose2d()));

    blueReefTags.add(Pair.of(22, layout.getTagPose(22).get().toPose2d()));

    blueReefTags.add(Pair.of(21, layout.getTagPose(21).get().toPose2d()));

    blueReefTags.add(Pair.of(20, layout.getTagPose(20).get().toPose2d()));

    blueReefTags.add(Pair.of(19, layout.getTagPose(19).get().toPose2d()));

    redReefTags.add(Pair.of(7, layout.getTagPose(7).get().toPose2d()));

    redReefTags.add(Pair.of(8, layout.getTagPose(8).get().toPose2d()));

    redReefTags.add(Pair.of(9, layout.getTagPose(9).get().toPose2d()));

    redReefTags.add(Pair.of(10, layout.getTagPose(10).get().toPose2d()));

    redReefTags.add(Pair.of(11, layout.getTagPose(11).get().toPose2d()));

    redReefTags.add(Pair.of(6, layout.getTagPose(6).get().toPose2d()));

    allReefTags.add(Pair.of(18, layout.getTagPose(18).get().toPose2d()));
    allReefTags.add(Pair.of(17, layout.getTagPose(17).get().toPose2d()));
    allReefTags.add(Pair.of(22, layout.getTagPose(22).get().toPose2d()));
    allReefTags.add(Pair.of(21, layout.getTagPose(21).get().toPose2d()));
    allReefTags.add(Pair.of(20, layout.getTagPose(20).get().toPose2d()));
    allReefTags.add(Pair.of(19, layout.getTagPose(19).get().toPose2d()));
    allReefTags.add(Pair.of(7, layout.getTagPose(7).get().toPose2d()));
    allReefTags.add(Pair.of(8, layout.getTagPose(8).get().toPose2d()));
    allReefTags.add(Pair.of(9, layout.getTagPose(9).get().toPose2d()));
    allReefTags.add(Pair.of(10, layout.getTagPose(10).get().toPose2d()));
    allReefTags.add(Pair.of(11, layout.getTagPose(11).get().toPose2d()));
    allReefTags.add(Pair.of(6, layout.getTagPose(6).get().toPose2d()));
  }

  private AprilTagFieldLayout loadField() {
    LAYOUT_ALERT.set(false);

    var path = Preferences.getString("Field/Path", "");
    var layout = Preferences.getString("Field/Layout", "k2025ReefscapeWelded");

    if (path != null && !path.isBlank()) {
      try {
        return new AprilTagFieldLayout(path);
      } catch (IOException ex) {
        DriverStation.reportError(ex.getLocalizedMessage(), ex.getStackTrace());
        LAYOUT_ALERT.setText(String.format(LAYOUT_ALERT_FMT, layout));
        LAYOUT_ALERT.set(true);
      }
    }

    return AprilTagFieldLayout.loadField(AprilTagFields.valueOf(layout));
  }

  private Pose2d getTagPose(int tag) {
    return layout.getTagPose(tag).get().toPose2d();
  }

  public Pose2d BranchPose(int tag, Side side) {
    Transform2d transform;
    if (side == Side.LEFT) {
      transform = TagToLeftBranch;
    } else {
      transform = TagToRightBranch;
    }

    return getTagPose(tag).plus(transform);
  }

  public Pose2d RobotPose(int tag, Side side) {
    Transform2d transform;
    if (side == Side.LEFT) {
      transform = TagToLeftBranch.plus(BranchToRobot);
    } else {
      transform = TagToRightBranch.plus(BranchToRobot);
    }

    return getTagPose(tag).plus(transform);
  }

  public List<Pose2d> AllRobotPoses =
      List.of(
          RobotPose(6, Side.LEFT),
          RobotPose(6, Side.RIGHT),
          RobotPose(7, Side.LEFT),
          RobotPose(7, Side.RIGHT),
          RobotPose(8, Side.LEFT),
          RobotPose(8, Side.RIGHT),
          RobotPose(9, Side.LEFT),
          RobotPose(9, Side.RIGHT),
          RobotPose(10, Side.LEFT),
          RobotPose(10, Side.RIGHT),
          RobotPose(11, Side.LEFT),
          RobotPose(11, Side.RIGHT),
          RobotPose(17, Side.LEFT),
          RobotPose(17, Side.RIGHT),
          RobotPose(18, Side.LEFT),
          RobotPose(18, Side.RIGHT),
          RobotPose(19, Side.LEFT),
          RobotPose(19, Side.RIGHT),
          RobotPose(20, Side.LEFT),
          RobotPose(20, Side.RIGHT),
          RobotPose(21, Side.LEFT),
          RobotPose(21, Side.RIGHT),
          RobotPose(22, Side.LEFT),
          RobotPose(22, Side.RIGHT));

  private List<Pair<Integer, Pose2d>> getReefTags(Alliance alliance) {
    if (alliance == Alliance.Red) {
      return redReefTags;
    }

    return blueReefTags;
  }

  private double distance(Pose2d start, Pose2d end) {
    return start.getTranslation().getDistance(end.getTranslation());
  }

  public int getClosestReefTag(Pose2d pose, Alliance alliance) {
    List<Pair<Integer, Pose2d>> tags = getReefTags(alliance);

    Pair<Integer, Pose2d> closestTag =
        Collections.min(tags, Comparator.comparingDouble(tag -> distance(pose, tag.getSecond())));

    return closestTag.getFirst();
  }

  public int getClosestReefTag(Pose2d pose) {
    List<Pair<Integer, Pose2d>> tags = allReefTags;

    Pair<Integer, Pose2d> closestTag =
        Collections.min(tags, Comparator.comparingDouble(tag -> distance(pose, tag.getSecond())));

    return closestTag.getFirst();
  }

  public Pose2d getTargetPoseForCage(Translation2d cage) {
    return new Pose2d(cage, Rotation2d.kZero).plus(CageToRobot);
  }

  public Pose2d getTargetPoseForBarge(Translation2d bargeScoringLocation) {
    return new Pose2d(bargeScoringLocation, Rotation2d.kZero).plus(BargeToRobot);
  }
}
