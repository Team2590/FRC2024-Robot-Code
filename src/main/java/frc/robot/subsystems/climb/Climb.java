package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climb extends SubsystemBase {
  private ClimbIO io;
  private ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();

  public Climb(ClimbIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
  }

  /** Run the climb motors */
  public void run() {
    io.setVoltage(6);
  }

  /** Flip the climb hooks */
  public void flip() {
    if (io.getRotationCount() < -11) {
      io.stop();
    } else {
      run();
    }
  }

  /** Reset the rotation count on the climb motors */
  public void resetRotationCount() {
    io.resetRotationCount();
  }

  /** Stop the motors */
  public void setStopped() {
    io.stop();
  }
}
