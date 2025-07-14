package frc.robot.oi;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Preferences;

public enum StructureState {
  HOME(Inches.zero(), Degrees.of(100)),
  INTAKE_CORAL(Inches.zero(), Degrees.of(100)),
  SCORE_CORAL_L1(Inches.zero(), Degrees.of(100)),
  SCORE_CORAL_L2(Inches.zero(), Degrees.of(100)),
  SCORE_CORAL_L3(Inches.zero(), Degrees.of(100)),
  SCORE_CORAL_L4(Inches.zero(), Degrees.of(100)),
  INTAKE_ALGAE_FLOOR(Inches.zero(), Degrees.of(100)),
  SCORE_ALGAE_PROCESSOR(Inches.zero(), Degrees.of(100)),
  INTAKE_ALGAE_L2(Inches.zero(), Degrees.of(100)),
  INTAKE_ALGAE_L3(Inches.zero(), Degrees.of(100)),
  PRIME_INTAKE_ALGAE_L2(Inches.zero(), Degrees.of(100)),
  PRIME_INTAKE_ALGAE_L3(Inches.zero(), Degrees.of(100)),
  SCORE_ALGAE_BARGE(Inches.zero(), Degrees.of(100)),
  CLIMB(Inches.zero(), Degrees.of(100));

  private final String HeightKey;
  private final String PitchKey;
  private final double DefaultHeight;
  private final double DefaultPitch;

  private StructureState(Distance height, Angle pitch) {
    HeightKey = String.format("Structure/%s/Height", this);
    PitchKey = String.format("Structure/%s/Pitch", this);
    DefaultHeight = height.in(Inches);
    DefaultPitch = pitch.in(Degrees);

    Preferences.initDouble(HeightKey, DefaultHeight);
    Preferences.initDouble(PitchKey, DefaultPitch);
  }

  public final Distance ElevatorHeight() {
    return Inches.of(Preferences.getDouble(HeightKey, DefaultHeight));
  }

  public final Angle ArmPitch() {
    return Degrees.of(Preferences.getDouble(PitchKey, DefaultPitch));
  }

  public boolean isAlgaeConfig() {
    switch (this) {
      case INTAKE_ALGAE_FLOOR:
      case SCORE_ALGAE_PROCESSOR:
      case INTAKE_ALGAE_L2:
      case INTAKE_ALGAE_L3:
      case SCORE_ALGAE_BARGE:
      case PRIME_INTAKE_ALGAE_L2:
      case PRIME_INTAKE_ALGAE_L3:
        return true;
      case HOME:
      case INTAKE_CORAL:
      case SCORE_CORAL_L1:
      case SCORE_CORAL_L2:
      case SCORE_CORAL_L3:
      case SCORE_CORAL_L4:
      case CLIMB:
      default:
        return false;
    }
  }

  public boolean isScoreCoralConfig() {
    switch (this) {
      case SCORE_CORAL_L1:
      case SCORE_CORAL_L2:
      case SCORE_CORAL_L3:
      case SCORE_CORAL_L4:
        return true;
      case HOME:
      case INTAKE_CORAL:
      case INTAKE_ALGAE_FLOOR:
      case SCORE_ALGAE_PROCESSOR:
      case INTAKE_ALGAE_L2:
      case INTAKE_ALGAE_L3:
      case PRIME_INTAKE_ALGAE_L2:
      case PRIME_INTAKE_ALGAE_L3:
      case SCORE_ALGAE_BARGE:
      case CLIMB:
      default:
        return false;
    }
  }

  public boolean isScoreAlgaeConfig() {
    switch (this) {
      case SCORE_ALGAE_PROCESSOR:
      case SCORE_ALGAE_BARGE:
        return true;
      case HOME:
      case INTAKE_CORAL:
      case INTAKE_ALGAE_FLOOR:
      case SCORE_CORAL_L1:
      case SCORE_CORAL_L2:
      case SCORE_CORAL_L3:
      case SCORE_CORAL_L4:
      case PRIME_INTAKE_ALGAE_L2:
      case PRIME_INTAKE_ALGAE_L3:
      case INTAKE_ALGAE_L2:
      case INTAKE_ALGAE_L3:
      case CLIMB:
      default:
        return false;
    }
  }

  public boolean isScoringConfig() {
    switch (this) {
      case SCORE_CORAL_L1:
      case SCORE_CORAL_L2:
      case SCORE_CORAL_L3:
      case SCORE_CORAL_L4:
      case SCORE_ALGAE_BARGE:
      case SCORE_ALGAE_PROCESSOR:
        return true;
      case HOME:
      case INTAKE_CORAL:
      case INTAKE_ALGAE_FLOOR:
      case PRIME_INTAKE_ALGAE_L2:
      case PRIME_INTAKE_ALGAE_L3:
      case INTAKE_ALGAE_L2:
      case INTAKE_ALGAE_L3:
      case CLIMB:
      default:
        return false;
    }
  }

  public boolean isReefConfig() {
    switch (this) {
      case SCORE_CORAL_L1:
      case SCORE_CORAL_L2:
      case SCORE_CORAL_L3:
      case SCORE_CORAL_L4:
      case PRIME_INTAKE_ALGAE_L2:
      case PRIME_INTAKE_ALGAE_L3:
      case INTAKE_ALGAE_L2:
      case INTAKE_ALGAE_L3:
        return true;
      case HOME:
      case INTAKE_CORAL:
      case INTAKE_ALGAE_FLOOR:
      case SCORE_ALGAE_BARGE:
      case SCORE_ALGAE_PROCESSOR:
      case CLIMB:
      default:
        return false;
    }
  }

  public boolean isHomeConfig() {
    switch (this) {
      case HOME:
      case INTAKE_CORAL:
      case INTAKE_ALGAE_FLOOR:
      case SCORE_ALGAE_PROCESSOR:
      case CLIMB:
        return true;
      case SCORE_CORAL_L1:
      case SCORE_CORAL_L2:
      case SCORE_CORAL_L3:
      case SCORE_CORAL_L4:
      case PRIME_INTAKE_ALGAE_L2:
      case PRIME_INTAKE_ALGAE_L3:
      case INTAKE_ALGAE_L2:
      case INTAKE_ALGAE_L3:
      case SCORE_ALGAE_BARGE:
      default:
        return false;
    }
  }

  public boolean requiresBumpUp() {
    switch (this) {
      case SCORE_CORAL_L1:
      case SCORE_CORAL_L4:
        return true;
      case SCORE_CORAL_L2:
      case SCORE_CORAL_L3:
      case PRIME_INTAKE_ALGAE_L2:
      case PRIME_INTAKE_ALGAE_L3:
      case INTAKE_ALGAE_L2:
      case INTAKE_ALGAE_L3:
      case SCORE_ALGAE_BARGE:
      case HOME:
      case INTAKE_CORAL:
      case INTAKE_ALGAE_FLOOR:
      case SCORE_ALGAE_PROCESSOR:
      case CLIMB:
      default:
        return false;
    }
  }
}
