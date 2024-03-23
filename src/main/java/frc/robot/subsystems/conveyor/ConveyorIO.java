package frc.robot.subsystems.conveyor;

import org.littletonrobotics.junction.AutoLog;

/**
 * @author Ian Keller
 */
public interface ConveyorIO {
  @AutoLog
  public static class ConveyorIOInputs {
    public boolean detectedShooterSide = false;
    public boolean hasNote = false;
    public double diverterRPM = 0;
    public double feederRPM = 0;
    public double diverterVolts = 0;
    public double feederVolts = 0;
    public double shooterProxVolts = 0.0;
    public double[] currentAmps = new double[] {};
  }

  /** Updates the set of loggable inputs, and applies them to the motors based on state. */
  public default void updateInputs(ConveyorIOInputs inputs) {}

  /**
   * Run the motors with the given power
   *
   * @param power - motor power
   */
  public default void runPower(double power) {}

  /**
   * Run the the motors with different given power
   *
   * @param feederPower - feeder motor power
   * @param diverterPower - diverter motor power
   */
  public default void runPower(double feederPower, double diverterPower) {}

  public default void stop() {}

  public default void setCoastMode() {}
}
