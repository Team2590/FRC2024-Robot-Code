package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import frc.robot.Constants.ClimbConstants;

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
    boolean leaderOutOfBounds = Math.abs(inputs.leaderPositionRotations) > ClimbConstants.CLIMB_POSITION_TARGET;
    boolean followerOutOfBounds = Math.abs(inputs.followerPositionRotations) > ClimbConstants.CLIMB_POSITION_TARGET;

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
        io.setVoltage(4); // 6 if using fast hooks
      }
    }
  }

  public void flip() {
    Logger.recordOutput("Climb/ClimberRotationL", inputs.leaderPositionRotations);
    Logger.recordOutput("Climb/ClimberRotationF", inputs.followerPositionRotations);

    boolean leaderOutOfBounds = Math.abs(inputs.leaderPositionRotations) > ClimbConstants.FLIP_POSITION_TARGET;
    boolean followerOutOfBounds = Math.abs(inputs.followerPositionRotations) > ClimbConstants.FLIP_POSITION_TARGET;

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
        io.setVoltage(3);
      }
    }
  }

  public void setCoast(){
    io.setCoast();
  }
  public void resetRotationCount() {
    io.resetRotationCount();
  }

  public void setStopped() {
    io.stop();
  }
}
