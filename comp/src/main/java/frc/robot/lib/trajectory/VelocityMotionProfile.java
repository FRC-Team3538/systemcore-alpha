package frc.robot.lib.trajectory;

// motion profile only looking to close on velocity, not full state
public class VelocityMotionProfile<T extends MotionCurve<T>> extends MotionProfile<T> {
  private final MotionCurve.Constraints<T> m_constraints;

  public VelocityMotionProfile(MotionCurve.Constraints<T> constraints) {
    m_constraints = constraints;
  }

  public State calculate(double t, State current, State goal) {
    var direction = shouldFlipInput(current, goal);

    var curve = m_constraints.throughState(current, direction);

    var timeToGoal = curve.timeToState(goal);

    if (timeToGoal <= t) {
      var distance = curve.computeDistanceFromVelocity(goal.velocity);
      distance += (t - timeToGoal) * goal.velocity;
      return new State(distance, goal.velocity);
    }

    return curve.stateAtTime(t);
  }

  public double timeRemaining(State current, State goal) {
    var direction = shouldFlipInput(current, goal);

    var curve = m_constraints.throughState(current, direction);

    return curve.timeToState(goal);
  }

  public boolean shouldFlipInput(State current, State goal) {
    return current.velocity > goal.velocity;
  }
}
