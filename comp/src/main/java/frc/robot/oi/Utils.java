package frc.robot.oi;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import java.util.function.Function;

public class Utils {
  /**
   * Scales an input axis according
   *
   * @param axis input axis
   * @param deadband deadband where [-deadband, deadband] is mapped to 0
   * @param map function mapping the input axis from [-1, 1] to the desired range
   * @return the mapped value
   */
  public static double scaleAxis(double axis, double deadband, Function<Double, Double> map) {
    axis = MathUtil.clamp(axis, -1, 1);
    axis = MathUtil.applyDeadband(axis, deadband);

    return map.apply(axis);
  }

  /**
   * Scales an input axis according
   *
   * @param axis input axis, range [-1, 1]
   * @param deadband deadband where [-deadband, deadband] is mapped to 0
   * @param power power to raise the input value to
   * @param scalar maximum value to map the input to
   * @return the mapped axis
   */
  public static double scaleAxis(double axis, double deadband, double power, double scalar) {
    return scaleAxis(axis, deadband, (value) -> Math.pow(value, power) * scalar);
  }

  /**
   * Scales an input axis according
   *
   * @param axis input axis, range [-1, 1]
   * @param deadband deadband where [-deadband, deadband] is mapped to 0
   * @param map An interpolation tree to map the input with
   * @return the mapped axis
   */
  public static double scaleAxis(double axis, double deadband, InterpolatingDoubleTreeMap map) {
    return scaleAxis(axis, deadband, (value) -> map.get(value));
  }
}
