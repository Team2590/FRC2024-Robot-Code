package frc.robot.subsystems.elevatorarm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
  @AutoLog
  public static class ArmIOInputs {
    public double velDegreesPerSecond = 0.0;
    public double currentAmps = 0.0;
    public double appliedVoltage = 0.0;
    public double appliedPercent = 0.0;
    public double armabspos = 0.0;
    public double armpos = 0.0;
  }

  /** Updates the set of loggable inputs */
  public default void updateInputs(ArmIOInputs inputs) {}

  /** Sets the desired angle of the wrist in degrees per second */
  public default void setAngle(double angleDegrees) {}

  /** Sets the speed of the wrist to the desired percent output */
  public default void setPercent(double percent) {}

  /** Resets the arm based on whatever encoder it has */
  public default void resetArm() {}

  /** Enables or disables the wrist in brake mode */
  public default void enableBrakeMode(boolean enable) {}

  /** Updates tunable numbers */
  public default void updateTunableNumbers() {}
}
