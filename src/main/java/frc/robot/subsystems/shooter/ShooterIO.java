package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    // public double positionRad = 0.0;
    // public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    // public double[] currentAmps = new double[] {};
  }

  public void updateInputs(ShooterIOInputs inputs);

  public void setVoltage(double voltage);

  public void setVelocity(double velocityRadPerSec);

  public void stop();
}
