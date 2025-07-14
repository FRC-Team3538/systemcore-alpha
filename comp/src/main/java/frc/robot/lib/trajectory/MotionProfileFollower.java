package frc.robot.lib.trajectory;

import frc.robot.lib.trajectory.MotionProfile.State;

public class MotionProfileFollower<P extends MotionProfile<?>> {
  private final P profile;
  private final State currentState = new State();
  private final State goalState = new State();

  public MotionProfileFollower(P profile) {
    this.profile = profile;
  }

  public State getCurrentState() {
    return new State(currentState.position, currentState.velocity);
  }

  public State getGoalState() {
    return new State(goalState.position, goalState.velocity);
  }

  public void setCurrentState(State current) {
    this.currentState.position = current.position;
    this.currentState.velocity = current.velocity;
  }

  public void setGoalState(State goal) {
    this.goalState.position = goal.position;
    this.goalState.velocity = goal.velocity;
  }

  public State update(double dt) {
    var state = profile.calculate(dt, currentState, goalState);
    currentState.position = state.position;
    currentState.velocity = state.velocity;

    return state;
  }

  public double timeRemaining() {
    return profile.timeRemaining(currentState, goalState);
  }
}
