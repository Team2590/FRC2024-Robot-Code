package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.AutoLog;

public interface ClimbIO {
  @AutoLog
  public class ClimbIOInputs {
    public double positionRotations = 0.0;
  }

  public void updateInputs(ClimbIOInputs inputs);

  public void run(double speed);

  public void setVoltage(double voltage);

  public void stop();

  public double getRotationCount();

  public void resetRotationCount();
}
