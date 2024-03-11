package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.Logger;

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
    io.setVoltage(4);
  }

  public void flip() {
    Logger.recordOutput("Climb/ClimberRotation", inputs.leaderPositionRotations);
    Logger.recordOutput("Climb/ClimberRotation", inputs.followerPositionRotations);

    boolean leaderOutOfBounds = Math.abs(inputs.leaderPositionRotations) > 11;
    boolean followerOutOfBounds = Math.abs(inputs.followerPositionRotations) > 11;

    // If both motors are above the threshold, stop both.
    if (leaderOutOfBounds && followerOutOfBounds) {
        io.stop();
    } else {
        // If only one motor is out of bounds, stop that specific motor.
        if (leaderOutOfBounds) {
            io.stopLeader();
        }
        if (followerOutOfBounds) {
            io.stopFollower();
        }

        // If neither motor is out of bounds, proceed to run them.
        if (!leaderOutOfBounds && !followerOutOfBounds) {
            io.setVoltage(4);
        }
    }
}


  public void resetRotationCount() {
    io.resetRotationCount();
  }

  public void setStopped() {
    io.stop();
  }
}
