/**
 * @author Dhruv and Shashank
 */
package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
  @AutoLog
  public static class ArmIOInputs {
    public double appliedPosition = 0.0;
    public double desiredPosition = 0.0;
    public double absolutePosition = 0.0;
    public double appliedVolts = 0.0;
  }

  public void updateInputs(ArmIOInputs inputs);

  public void setPosition(double positionRad);

  public void stop();
}
