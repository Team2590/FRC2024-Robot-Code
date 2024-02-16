package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climb extends SubsystemBase {
  private ClimbIO io;
  private ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();
  private ClimbStates state;

  public Climb(ClimbIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
  }

  public void up() {
    io.up();
    state = ClimbStates.MANUAL;
  }

  public void down() {
    io.down();
    state = ClimbStates.MANUAL;
  }

  public void stop() {
    io.stop();
    state = ClimbStates.STOPPED;
  }

  public void resetRotationCount() {
    io.resetRotationCount();
    state = ClimbStates.MANUAL;
  }

  public ClimbStates getState() {
    return state;
  }

  public void setState(ClimbStates newState) {
    state = newState;
  }
}
