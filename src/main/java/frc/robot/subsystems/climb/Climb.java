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
    io.setVoltage(6);
  }

  public void flip() {
    if (io.getRotationCount() < -11) {
      io.stop();
    } else {
      run();
    }
  }

  public void autoFlip() {
    System.out.println("Auto flip running");
    if (io.getRotationCount() > -11) {
      run();
      autoFlip();
    } else {
      io.stop();
    }
  }

  public void resetRotationCount() {
    io.resetRotationCount();
  }

  public void setStopped() {
    io.stop();
  }
}
