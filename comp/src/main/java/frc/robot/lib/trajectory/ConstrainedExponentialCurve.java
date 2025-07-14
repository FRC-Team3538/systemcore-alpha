package frc.robot.lib.trajectory;

import frc.robot.lib.trajectory.MotionProfile.State;

public class ConstrainedExponentialCurve extends MotionCurve<ConstrainedExponentialCurve> {
  private final Constraints m_constraints;
  private final State m_initialState;
  private final State m_switchingState;
  private final TrapezoidCurve m_trapezoid;
  private final ExponentialCurve m_exponential;
  private final double m_timeToSwitchingState;
  private final boolean m_direction;

  private ConstrainedExponentialCurve(
      Constraints constraints, State switchingState, State initialState, boolean direction) {
    m_constraints = constraints;
    m_trapezoid = constraints.m_trapezoid.throughState(switchingState, direction);
    m_exponential = constraints.m_exponential.throughState(switchingState, direction);
    m_initialState = initialState;
    m_switchingState = switchingState;
    m_direction = direction;

    if (m_initialState.velocity > m_switchingState.velocity) {
      m_timeToSwitchingState = -m_exponential.timeToState(initialState);
    } else {
      m_timeToSwitchingState = -m_trapezoid.timeToState(initialState);
    }
  }

  public static class Constraints
      extends frc.robot.lib.trajectory.MotionCurve.Constraints<ConstrainedExponentialCurve> {
    private final TrapezoidCurve.Constraints m_trapezoid;
    private final ExponentialCurve.Constraints m_exponential;

    private Constraints(
        TrapezoidCurve.Constraints trapezoid, ExponentialCurve.Constraints exponential) {
      m_trapezoid = trapezoid;
      m_exponential = exponential;
    }

    public ConstrainedExponentialCurve throughState(State state, boolean direction) {
      var switchingVelocity = SwitchingVelocity();
      double switchingPosition;

      if (state.velocity <= SwitchingVelocity()) {
        switchingPosition =
            m_trapezoid
                .throughState(state, direction)
                .computeDistanceFromVelocity(switchingVelocity);
      } else {
        switchingPosition =
            m_exponential
                .throughState(state, direction)
                .computeDistanceFromVelocity(switchingVelocity);
      }

      return new ConstrainedExponentialCurve(
          this, new State(switchingVelocity, switchingPosition), state, direction);
    }

    public double SwitchingVelocity() {
      return m_exponential.MaxAchievableVelocity(m_trapezoid.maxAcceleration);
    }

    @Override
    public Constraints withMaxVelocity(double velocity) {
      super.withMaxVelocity(velocity);

      return this;
    }

    @Override
    public ConstrainedExponentialProfile asFullStateMotionProfile() {
      throw new UnsupportedOperationException();
    }

    @Override
    public VelocityMotionProfile<ConstrainedExponentialCurve> asVelocityMotionProfile() {
      return new VelocityMotionProfile<>(this);
    }
  }

  @Override
  public double computeDistanceFromVelocity(double velocity) {
    if (Math.abs(velocity - m_switchingState.velocity) > 0) {
      return m_exponential.computeDistanceFromVelocity(velocity);
    }

    return m_trapezoid.computeDistanceFromVelocity(velocity);
  }

  @Override
  public double timeToState(State goal) {
    if (Math.abs(goal.velocity - m_switchingState.velocity) > 0) {
      return m_exponential.timeToState(goal) + m_timeToSwitchingState;
    }

    return m_trapezoid.timeToState(goal) + m_timeToSwitchingState;
  }

  @Override
  public double computeVelocityFromTime(double t) {
    if (t > m_timeToSwitchingState) {
      return m_exponential.computeVelocityFromTime(t - m_timeToSwitchingState);
    }

    return m_trapezoid.computeVelocityFromTime(t);
  }

  @Override
  public double computeDistanceFromTime(double t) {
    if (t > m_timeToSwitchingState) {
      return m_exponential.computeDistanceFromTime(t - m_timeToSwitchingState);
    }

    return m_trapezoid.computeDistanceFromTime(t);
  }

  @Override
  public double intersectionVelocity(ConstrainedExponentialCurve second) {
    return 0;
  }
}
