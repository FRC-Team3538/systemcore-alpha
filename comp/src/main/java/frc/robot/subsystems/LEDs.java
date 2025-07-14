package frc.robot.subsystems;


import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.LEDConfig;
import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.controls.SingleFadeAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;

public class LEDs extends SubsystemBase {
  private CANdle candle;
  private final LEDConfig config;

  public LEDs(LEDConfig config) {
    this.config = config;
    this.candle = config.CANdleID().getLED();
    this.candle.getConfigurator().apply(config.CANdleConfig());

    setColor(config.CycleModeColor());
  }

  public Command SetColorCommand(Color color) {
    return run(() -> setColor(color))
        .withName(
            String.format(
                "LED::SetColor(%s, %s, %s)",
                (int) (color.red * 255), (int) (color.green * 255), (int) (color.blue * 255)));
  }

  private void setColor(Color color) {
    SolidColor animation = new SolidColor(0,-1).withColor(new RGBWColor(color));
    candle.setControl(animation);
  }

  public Command ClimberModeCommand() {
    return SetColorCommand(config.ClimbModeColor())
        .ignoringDisable(true)
        .withName("LED::ClimbMode");
  }

  public Command CyclingModeCommand(BooleanSupplier defenseMode) {
    return run(() -> {
          if (defenseMode.getAsBoolean()) {
            setColor(config.DefenseModeColor());
          } else {
            setColor(config.CycleModeColor());
          }
        })
        .ignoringDisable(true)
        .withName("LED::CycleMode");
  }

  public Command BlinkColorCommand(Color color) {
    StrobeAnimation strobe =
        new StrobeAnimation(0,-1).withColor(new RGBWColor(color)).withFrameRate(0.25);
    return run(() -> {
          candle.setControl(strobe);
        })
        .withName(
            String.format(
                "LED::BlinkColor(%s, %s, %s)",
                (int) (color.red * 255), (int) (color.green * 255), (int) (color.blue * 255)));
  }

  public Command FadeColorCommand(Color color) {
    SingleFadeAnimation fade =
        new SingleFadeAnimation(0, -1).withFrameRate(0.1).withColor(new RGBWColor(color));

    return run(() -> {
          candle.setControl(fade);
        })
        .withName(
            String.format(
                "LED::FadeColor(%s, %s, %s)",
                (int) (color.red * 255), (int) (color.green * 255), (int) (color.blue * 255)));
  }

  public Command CloseRightAlignmentCommand() {
    return SetColorCommand(Color.kGreen).withName("LED::CloseRightAlignment").asProxy();
  }

  public Command FarRightAlignmentCommand() {
    return SetColorCommand(Color.kYellow).withName("LED::FarRightAlignment").asProxy();
  }

  public Command FarFaceAlignmentCommand() {
    return SetColorCommand(Color.kRed).withName("LED::FarFaceAlignment").asProxy();
  }

  public Command FarLeftAlignmentCommand() {
    return SetColorCommand(Color.kMagenta).withName("LED::FarLeftAlignment").asProxy();
  }

  public Command CloseLeftAlignmentCommand() {
    return SetColorCommand(Color.kBlue).withName("LED::CloseLeftAlignment").asProxy();
  }

  public Command CloseFaceAlignmentCommand() {
    return SetColorCommand(Color.kCyan).withName("LED::CloseFaceAlignment").asProxy();
  }

  public Command CoralIntaking() {
    return BlinkColorCommand(Color.kRed).withName("LED::CoralIntaking").asProxy();
  }

  public Command AlgaeIntaking() {
    return SetColorCommand(Color.kTeal).withName("LED::AlgaeIntaking").asProxy();
  }

  public Command AlgaeDetected() {
    return BlinkColorCommand(Color.kTeal).withName("LED::AlgaeDetected").asProxy();
  }

  public Command CoralDetected() {
    return BlinkColorCommand(Color.kGhostWhite).withName("LED::CoralDetected").asProxy();
  }

  public Command CoralScored() {
    return SetColorCommand(Color.kWhite).withName("LED::CoralDetected").asProxy();
  }

  public Command AlgaeScored() {
    return BlinkColorCommand(Color.kHotPink).withName("LED::CoralDetected").asProxy();
  }

  public Command BargeSafe() {
    return SetColorCommand(Color.kGreen).withName("LED::IsAlignedBarge").asProxy();
  }

  public Command BargeUnsafe() {
    return BlinkColorCommand(Color.kRed).withName("LED::IsNotAlignedToBarge").asProxy();
  }
}
