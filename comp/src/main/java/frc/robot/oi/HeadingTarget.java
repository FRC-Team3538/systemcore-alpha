package frc.robot.oi;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.constants.FieldGeometry;
import frc.robot.lib.AllianceFlipUtil;

public enum HeadingTarget {
  NORTH,
  SOUTH,
  EAST,
  WEST,
  INTAKE,
  LEFT,
  RIGHT,
  FORWARD,
  BACKWARD,
  NONE;

  public Rotation2d getHeading(Alliance alliance, Pose2d currentPose) {
    switch (this) {
      case INTAKE:
        // +Y is leftward
        if (currentPose.getY() > FieldGeometry.fieldWidth / 2) {
          return AllianceFlipUtil.mirror(_INTAKE_LEFT);
        }
        return AllianceFlipUtil.mirror(_INTAKE_RIGHT);
      case NORTH:
        return AllianceFlipUtil.rotate(_NORTH);
      case SOUTH:
        return AllianceFlipUtil.rotate(_SOUTH);
      case EAST:
        return AllianceFlipUtil.rotate(_EAST);
      case WEST:
        return AllianceFlipUtil.rotate(_WEST);
      case FORWARD:
        return AllianceFlipUtil.rotate(Rotation2d.kZero);
      case BACKWARD:
        return AllianceFlipUtil.rotate(Rotation2d.k180deg);
      case LEFT:
        return AllianceFlipUtil.rotate(Rotation2d.kCCW_90deg);
      case RIGHT:
        return AllianceFlipUtil.rotate(Rotation2d.kCW_90deg);
      default:
        // This is never called for other cases - ignore
        return Rotation2d.fromDegrees(0);
    }
  }

  private static final Rotation2d _INTAKE_LEFT =
      FieldGeometry.CoralStation.leftCenterFace.getRotation();
  private static final Rotation2d _INTAKE_RIGHT =
      FieldGeometry.CoralStation.rightCenterFace.getRotation();
  private static final Rotation2d _NORTH = Rotation2d.kZero;
  private static final Rotation2d _SOUTH = Rotation2d.k180deg;
  private static final Rotation2d _EAST = Rotation2d.kCW_90deg;
  private static final Rotation2d _WEST = Rotation2d.kCCW_90deg;
}
