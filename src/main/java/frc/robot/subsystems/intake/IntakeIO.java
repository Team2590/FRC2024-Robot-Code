package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

  @AutoLog
  public static class IntakeIOInputs {
    public double VELOCITY = 0.0;
    public double APPLIEDVOLTS = 0.0;
    public double CURRENT = 0.0;
    public double POSITION = 0.0;
  }

  public default void setPower(double powerPercent) {}

  public default void stop() {}

  public default void setVoltage() {}

  public default void updateInputs(IntakeIOInputs inputs) {}
}
