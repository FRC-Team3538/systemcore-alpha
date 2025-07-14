package frc.robot.lib.trajectory;

import java.util.Objects;

public abstract class MotionProfile<T extends MotionCurve<T>> {
  public static class State {
    public double position;

    public double velocity;

    public State() {}

    public State(double position, double velocity) {
      this.position = position;
      this.velocity = velocity;
    }

    @Override
    public boolean equals(Object other) {
      if (other instanceof State) {
        State rhs = (State) other;
        return this.position == rhs.position && this.velocity == rhs.velocity;
      } else {
        return false;
      }
    }

    @Override
    public int hashCode() {
      return Objects.hash(position, velocity);
    }

    @Override
    public String toString() {
      return String.format("State(%s, %s)", position, velocity);
    }
  }

  public abstract State calculate(double t, State current, State goal);

  public abstract double timeRemaining(State current, State goal);

  public abstract boolean shouldFlipInput(State current, State goal);

  public MotionProfileFollower<MotionProfile<T>> follower() {
    return new MotionProfileFollower<MotionProfile<T>>(this);
  }

  public RotationMotionProfileFollower<MotionProfile<T>> rotationFollower() {
    return new RotationMotionProfileFollower<MotionProfile<T>>(this);
  }
}
