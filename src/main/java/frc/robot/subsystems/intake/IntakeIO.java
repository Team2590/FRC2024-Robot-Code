package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

  @AutoLog
  public static class IntakeIOInputs {
    public double velocity = 0.0;
    public double appliedVolts = 0.0;
    public double current = 0.0;
    public double position = 0.0;
  }

  public default void setPower(double powerPercent) {}

  public default void stop() {}

  public default void setVoltage() {}

  public default void updateInputs(IntakeIOInputs inputs) {}
}
