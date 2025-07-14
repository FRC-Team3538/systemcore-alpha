package frc.robot.lib.trajectory;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.lib.trajectory.MotionProfile.State;

public class RotationMotionProfileFollower<P extends MotionProfile<?>> {
  private final P profile;
  private final State currentState = new State();
  private final State goalState = new State();

  public RotationMotionProfileFollower(P profile) {
    this.profile = profile;
  }

  public State getCurrentState() {
    return new State(currentState.position, currentState.velocity);
  }

  public State getGoalState() {
    return new State(goalState.position, goalState.velocity);
  }

  public void setCurrentState(Rotation2d position, double velocity) {
    this.currentState.position = position.getRadians();
    this.currentState.velocity = velocity;

    fixCurrent();
  }

  public void setGoalState(Rotation2d position, double velocity) {
    this.goalState.position = position.getRadians();
    this.goalState.velocity = velocity;

    fixCurrent();
  }

  public State update(double dt) {
    var state = profile.calculate(dt, currentState, goalState);
    currentState.position = state.position;
    currentState.velocity = state.velocity;

    return state;
  }

  private boolean fixCurrent() {
    double currentTime = timeRemaining();

    double adjustment = 0;
    if (goalState.position >= currentState.position) {
      adjustment = 2 * Math.PI;
    } else {
      adjustment -= 2 * Math.PI;
    }
    currentState.position += adjustment;

    if (timeRemaining() < currentTime) {
      return true;
    }

    currentState.position -= adjustment;
    return false;
  }

  public double timeRemaining() {
    return profile.timeRemaining(currentState, goalState);
  }
}
