/**
 * @author Dhruv and Shashank
 */
package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    public double appliedVolts = 0.0;
    public double RPM = 0.0;
    public double desiredVolts = 0.0;
  }

  public void updateInputs(ShooterIOInputs inputs);

  // public void setVoltage(double voltage);

  public void setVelocity(double velocityRadPerSec);

  public void stop();
}
