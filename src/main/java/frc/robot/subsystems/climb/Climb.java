package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class Climb extends SubsystemBase {
  private ClimbIO io;
  private ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();

  private enum States {
    DEFAULT,
    FLIPPING
  }

  private States state;

  public Climb(ClimbIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    if (state == States.FLIPPING && io.getRotationCount() < -2) {
      io.stop();
      state = States.DEFAULT;
    }
  }

  public void run() {
    if (io.getRotationCount() < ClimbConstants.MAX_ROTATIONS) {
      io.stop();
    } else {
      io.run(0.15);
    }
    io.run(0.15);
  }

  public void flip() {
    io.run(0.15);
    state = States.FLIPPING;
  }

  public void resetRotationCount() {
    io.resetRotationCount();
  }

  public void setStopped() {
    io.stop();
  }
}
