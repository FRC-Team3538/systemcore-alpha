package frc.robot.oi;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static java.lang.Math.cos;
import static java.lang.Math.sin;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Preferences;
import frc.robot.constants.Constants.ControlsConfig;
import frc.robot.lib.RJLog;

public class DriverControls {
  private final GuliKitController driver;
  public final ControlsConfig config;

  public DriverControls(GuliKitController driver, ControlsConfig config) {
    this.driver = driver;
    this.config = config;

    Preferences.initDouble("Controller/Driver/Left_Deadband", 0);
    Preferences.initDouble("Controller/Driver/Left_Limit", 1);
    Preferences.initDouble("Controller/Driver/Left_Exponent", 1);
    Preferences.initDouble("Controller/Driver/Right_Deadband", 0);
    Preferences.initDouble("Controller/Driver/Right_Limit", 1);
    Preferences.initDouble("Controller/Driver/Right_Exponent", 1);

    Preferences.initDouble("Controller/FieldCentric/Scalar", 1);
    Preferences.initDouble("Controller/Rotation/Scalar", 1);
    Preferences.initDouble("Controller/RobotCentric/Scalar", 1);
  }

  public ChassisSpeeds fieldCentric(ControlMode controlMode) {
    var translation =
        fieldCentricInternal().times(Preferences.getDouble("Controller/FieldCentric/Scalar", 1));
    var rotation = rotateInternal();

    return new ChassisSpeeds(
        translation.get(0) * config.MaxSpeed().in(MetersPerSecond),
        translation.get(1) * config.MaxSpeed().in(MetersPerSecond),
        rotation * config.MaxAngularRate().in(RadiansPerSecond));
  }

  public boolean shouldUseRobotCentric() {
    return driver.getPOV() != 0;
  }

  public boolean shouldManuallyRotate() {
    return !driver.getLeftUpperPaddleButton() && !driver.getRightUpperPaddleButton();
  }

  public ChassisSpeeds robotCentric(ControlMode controlMode) {
    var translation =
        robotCentricInternal().times(Preferences.getDouble("Controller/RobotCentric/Scalar", 1));
    var rotation = rotateInternal() * Preferences.getDouble("Controller/Rotation/scalar", 1);

    return new ChassisSpeeds(
        translation.get(0) * config.MaxSpeed().in(MetersPerSecond),
        translation.get(1) * config.MaxSpeed().in(MetersPerSecond),
        rotation * config.MaxAngularRate().in(RadiansPerSecond));
  }

  public double rotateInternal() {
    var deadband = Preferences.getDouble("Controller/Driver/Right_Deadband", 0);
    var limit = Preferences.getDouble("Controller/Driver/Right_Limit", 1);
    var power = Preferences.getDouble("Controller/Driver/Right_Exponent", 1);

    var rotate = -driver.getRightX();

    return Utils.scaleAxis(rotate, deadband, power, limit);
  }

  private Vector<N2> robotCentricInternal() {
    var pov = driver.getPOV();

    if (pov == 0) {
      return VecBuilder.fill(0, 0);
    } else {

      // i hate this
      var angles =
          new int[] {
            0, 0, // up
            90, // right
            45, // up right
            180, // down
            0, 135, // down right
            0, 270, // left
            315, // up left
            0, 0, 225, // down left
            0, 0, 0
          };

      pov = angles[pov];

      var angle = Units.Degrees.of(pov).in(Units.Radians);
      return VecBuilder.fill(cos(angle), sin(-angle));
    }
  }

  private Vector<N2> fieldCentricInternal() {
    var deadband = Preferences.getDouble("Controller/Driver/Left_Deadband", 0);
    var limit = Preferences.getDouble("Controller/Driver/Left_Limit", 1);
    var power = Preferences.getDouble("Controller/Driver/Left_Exponent", 1);

    var translate = VecBuilder.fill(-driver.getLeftY(), -driver.getLeftX());

    var magnitude = translate.norm();

    if (magnitude == 0) {
      return translate;
    }

    magnitude = Utils.scaleAxis(magnitude, deadband, power, limit);

    return translate.unit().times(magnitude);
  }

  public boolean intake() {
    return driver.getLeftBumperButton();
  }

  public boolean aim() {
    return driver.getLeftTriggerAxis() > 0.5;
  }

  public int pointToTagOnReef() {
    var face = rightStickBucketedAngle();

    RJLog.log("Controls/ReefAngle", face);

    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
      if (face == -150) {
        return 6;
      } else if (face == -90) {
        return 7;
      } else if (face == -30) {
        return 8;
      } else if (face == 30) {
        return 9;
      } else if (face == 90) {
        return 10;
      } else if (face == 150) {
        return 11;
      }
    } else {
      if (face == -150) {
        return 19;
      } else if (face == -90) {
        return 18;
      } else if (face == -30) {
        return 17;
      } else if (face == 30) {
        return 22;
      } else if (face == 90) {
        return 21;
      } else if (face == 150) {
        return 20;
      }
    }

    return 0;
  }

  public int rightStickBucketedAngle() {
    var rightStick = rightStickInternal();

    // If stick isn't pushed far, ignore
    if (rightStick.norm() < 0.8) {
      return -1;
    }

    // RJLog.log("Driver/RightY", driver.getRightY());
    // RJLog.log("Driver/RightX", driver.getRightX());

    var angle =
        new Rotation2d(rightStick.get(0), rightStick.get(1))
            .plus(Rotation2d.kCCW_90deg)
            .getDegrees();
    return 60 * (int) Math.round((angle - 30) / 60) + 30;
  }

  private Vector<N2> rightStickInternal() {
    var deadband = Preferences.getDouble("Controller/Driver/Right_Deadband", 0);
    var limit = Preferences.getDouble("Controller/Driver/Right_Limit", 1);
    var power = Preferences.getDouble("Controller/Driver/Right_Exponent", 1);

    var translate = VecBuilder.fill(-driver.getRightY(), -driver.getRightX());

    var magnitude = translate.norm();

    if (magnitude == 0) {
      return translate;
    }

    magnitude = Utils.scaleAxis(magnitude, deadband, power, limit);

    return translate.unit().times(magnitude);
  }
}
