package frc.robot.oi;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.Preferences;
import frc.robot.constants.Constants.ControlsConfig;

public class OperatorControls {
  private final PS4Controller operator;
  private final ControlsConfig config;

  public OperatorControls(PS4Controller operator, ControlsConfig config) {
    this.operator = operator;
    this.config = config;

    Preferences.initDouble("Controller/Operator/Left_Deadband", 0);
    Preferences.initDouble("Controller/Operator/Left_Limit", 1);
    Preferences.initDouble("Controller/Operator/Right_Deadband", 0);
    Preferences.initDouble("Controller/Operator/Right_Limit", 1);
  }

  public double manualElevator() {
    var deadband = Preferences.getDouble("Controller/Operator/Left_Deadband", 0);
    var limit = Preferences.getDouble("Controller/Operator/Left_Limit", 1);

    var manual = -operator.getLeftY();

    return Utils.scaleAxis(manual, deadband, 1, limit);
  }

  public double manualArm() {
    var deadband = Preferences.getDouble("Controller/Operator/Right_Deadband", 0);
    var limit = Preferences.getDouble("Controller/Operator/Right_Limit", 1);

    var manual = -operator.getRightY();

    return Utils.scaleAxis(manual, deadband, 1, limit);
  }

  public double climbertrigger() {
    return (operator.getL2Axis() - operator.getR2Axis()) / 2;
  }
}
