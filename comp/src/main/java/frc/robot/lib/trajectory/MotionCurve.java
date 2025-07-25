package frc.robot.lib.trajectory;

import frc.robot.lib.trajectory.MotionProfile.State;

public abstract class MotionCurve<T extends MotionCurve<T>> {
  public abstract static class Constraints<T extends MotionCurve<T>> {
    public double maxVelocity = Double.MAX_VALUE;

    public abstract T throughState(State state, boolean direction);

    public abstract FullStateMotionProfile<T> asFullStateMotionProfile();

    public abstract VelocityMotionProfile<T> asVelocityMotionProfile();

    public Constraints<T> withMaxVelocity(double velocity) {
      this.maxVelocity = velocity;

      return this;
    }
  }

  public abstract double computeDistanceFromVelocity(double velocity);

  public abstract double timeToState(State goal);

  public State stateAtTime(double t) {
    return new State(computeDistanceFromTime(t), computeVelocityFromTime(t));
  }

  public abstract double computeVelocityFromTime(double t);

  public abstract double computeDistanceFromTime(double t);

  public abstract double intersectionVelocity(T other);

  public State intersection(T other) {
    var velocity = intersectionVelocity(other);
    return new State(other.computeDistanceFromVelocity(velocity), velocity);
  }
}
