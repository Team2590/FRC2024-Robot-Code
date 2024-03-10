package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Climb extends SubsystemBase {
  private ClimbIO io;
  private ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();

  private enum States {
    AUTOFLIPPING,
    DEFAULT
  }

  private States state;

  public Climb(ClimbIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    // if (state == States.AUTOFLIPPING) {
    //   flip();
    // } else if (state == States.DEFAULT) {
    //   return;
    // } else {
    //   state = States.DEFAULT;
    //   io.stop();
    // }
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
    System.out.println("auto flip called");
    if (io.getRotationCount() < -11) {
      System.out.println("auto flip stopped");
      io.stop();
      return;
    } else {
      Logger.recordOutput("Climb/Climb", io.getRotationCount());
      System.out.println("auto flipping");
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
