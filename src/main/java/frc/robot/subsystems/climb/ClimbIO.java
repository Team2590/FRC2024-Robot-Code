package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.AutoLog;

public interface ClimbIO {
  @AutoLog
  public class ClimbIOInputs {
    public double leaderPositionRotations = 0.0;
    public double followerPositionRotations = 0.0;
  }

  public void updateInputs(ClimbIOInputs inputs);

  public void run(double speed);

  public void setVoltage(double voltage);

  public void stop();

  public double getRotationCount();

  public void resetRotationCount();

  public void stopLeader();

  public void stopFollower();
}
