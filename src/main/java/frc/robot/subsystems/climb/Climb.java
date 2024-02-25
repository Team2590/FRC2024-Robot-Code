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

  public void run() {
    // if (io.getRotationCount() < ClimbConstants.MAX_ROTATIONS) {
    //   io.stop();
    // } else {
    //   io.setVoltage(2);
    // }
    io.setVoltage(2);
  }

  public void flip() {
    if (io.getRotationCount() < -11) {
      io.stop();
    } else {
      run();
    }
  }

  public void resetRotationCount() {
    io.resetRotationCount();
  }

  public void setStopped() {
    io.stop();
  }
}
