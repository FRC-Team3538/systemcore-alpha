package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.Constants.ControlsConfig;

public class ControlModeManager extends SubsystemBase {
  private ControlMode currentMode;

  public ControlModeManager(ControlsConfig config) {
    currentMode = config.DefaultControlMode();
    setDefaultCommand(goToMode(currentMode));
  }

  public Trigger isMode(ControlMode mode) {
    return new Trigger(() -> mode.equals(currentMode));
  }

  public Command goToMode(ControlMode mode) {
    return run(() -> currentMode = mode)
        .ignoringDisable(true)
        .withName(String.format("ControlModeManager::Mode(%s)", mode));
  }

  public ControlMode getCurrentMode() {
    return currentMode;
  }
}
