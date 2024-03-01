package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.AutoLog;

public interface ClimbIO {
  @AutoLog
  public class ClimbIOInputs {
    public double positionRotations = 0.0;
  }

  /** Updates the set of loggable inputs */
  public default void updateInputs(ClimbIOInputs inputs) {}

  /** Run the motors at the give speed */
  public default void run(double speed) {}

  /** Run the motors at the given voltage. If one motor falls behind, the other will be slowed so it can catch up */
  public default void setVoltage(double voltage) {}

  /** Stop the motors */
  public default void stop() {}

  /** Returns the rotation count of the leading motor */
  public default double getRotationCount() { return 0; }

  /** Resets rotation count of both motors */
  public default void resetRotationCount() {}
}
