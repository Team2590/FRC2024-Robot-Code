package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterTalonFXIO {
    @AutoLog
    public static class ShooterTalonFXIOInputs {
        public double velocity = 0.0;
    }

    public void updateInputs(ShooterTalonFXIOInputs inputs);

    public void setVelocity(double velocity);

    public void stop(); 

    public void configurePID(double kP, double kI, double kD);
}
