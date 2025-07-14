package frc.robot.constants;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import java.util.function.Supplier;
import org.photonvision.simulation.SimCameraProperties;

public class VisionConstants {
  // AprilTag layout
  public static AprilTagFieldLayout aprilTagLayout = FieldConstants.getInstance().layout;

  // Camera names, must match names configured on coprocessor
  public static String camera0Name = "Front Left";
  public static String camera1Name = "Front Right";
  public static String camera2Name = "Back Left";
  public static String camera3Name = "Back Right";

  public static Supplier<SimCameraProperties> camera0SimProperties =
      () -> {
        var cameraProps = new SimCameraProperties();

        // A 640 x 480 camera with a 100 degree diagonal FOV.
        cameraProps.setCalibration(1280, 800, Rotation2d.fromDegrees(92));
        // Approximate detection noise with average and standard deviation error in pixels.
        cameraProps.setCalibError(0.25, 0.08);
        // Set the camera image capture framerate (Note: this is limited by robot loop rate).
        cameraProps.setFPS(120);
        // The average and standard deviation in milliseconds of image data latency.
        cameraProps.setAvgLatencyMs(35);
        cameraProps.setExposureTimeMs(16);
        cameraProps.setLatencyStdDevMs(5);

        return cameraProps;
      };

  // Robot to camera transforms
  // (Not used by Limelight, configure in web UI instead)
  public static Transform3d robotToCamera0 =
      new Transform3d(
          new Translation3d(Inches.of(11.25), Inches.of(12), Inches.of(8.5)),
          new Rotation3d(Degrees.zero(), Degrees.of(-22), Degrees.of(-36)));

  public static Transform3d robotToCamera1 =
      new Transform3d(
          new Translation3d(Inches.of(11.25), Inches.of(-12), Inches.of(8.5)),
          new Rotation3d(Degrees.zero(), Degrees.of(-22), Degrees.of(36)));

  public static Transform3d robotToCamera2 =
      new Transform3d(
          new Translation3d(Inches.of(-12), Inches.of(12), Inches.of(8.5)),
          new Rotation3d(Degrees.zero(), Degrees.of(-22), Degrees.of(-180 + 36)));

  public static Transform3d robotToCamera3 =
      new Transform3d(
          new Translation3d(Inches.of(-12), Inches.of(-12), Inches.of(8.5)),
          new Rotation3d(Degrees.zero(), Degrees.of(-22), Degrees.of(180 - 36)));

  // Basic filtering thresholds
  public static double maxAmbiguity = 0.3;
  public static double maxZError = 0.75;

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static double linearStdDevBaseline = 0.02; // Meters
  public static double angularStdDevBaseline = 0.06; // Radians

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  public static double[] cameraStdDevFactors =
      new double[] {
        1.0, // Camera 0
        1.0 // Camera 1
      };

  // Multipliers to apply for MegaTag 2 observations
  public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
  public static double angularStdDevMegatag2Factor =
      Double.POSITIVE_INFINITY; // No rotation data available
}
