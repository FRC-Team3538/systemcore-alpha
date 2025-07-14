package frc.robot.auto;

public record Setpoint(String value) {
  public static final Setpoint START = new Setpoint("Start");
  public static final Setpoint SL = new Setpoint("SL");
  public static final Setpoint SR = new Setpoint("SR");
  public static final Setpoint A = new Setpoint("A");
  public static final Setpoint B = new Setpoint("B");
  public static final Setpoint C = new Setpoint("C");
  public static final Setpoint D = new Setpoint("D");
  public static final Setpoint E = new Setpoint("E");
  public static final Setpoint F = new Setpoint("F");
  public static final Setpoint G = new Setpoint("G");
  public static final Setpoint H = new Setpoint("H");
  public static final Setpoint I = new Setpoint("I");
  public static final Setpoint J = new Setpoint("J");
  public static final Setpoint K = new Setpoint("K");
  public static final Setpoint L = new Setpoint("L");

  public String to(Setpoint other) {
    return String.format("%s-%s", this.value(), other.value());
  }
}
