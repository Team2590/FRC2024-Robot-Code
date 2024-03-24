package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

  @AutoLog
  public static class IntakeIOInputs {
    public double leaderVelocity = 0.0;
    public double leaderAppliedVolts = 0.0;
    public double leaderCurrent = 0.0;
    public double leaderPosition = 0.0;
    public double followerVelocity = 0.0;
    public double followerAppliedVolts = 0.0;
    public double followerCurrent = 0.0;
    public double followerPosition = 0.0;
  }

  public default void setPower(double powerPercent) {}

  public default void stop() {}

  public default void setVoltage() {}

  public default void updateInputs(IntakeIOInputs inputs) {}
}
