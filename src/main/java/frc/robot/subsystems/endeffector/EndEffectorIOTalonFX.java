package frc.robot.subsystems.endeffector;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class EndEffectorIOTalonFX implements EndEffectorIO {
  private final TalonFX motor;

  public EndEffectorIOTalonFX() {
    motor = new TalonFX(2);
    motor.setNeutralMode(NeutralModeValue.Brake);
  }

  @Override
  public void updateInputs(EndEffectorIOInputs inputs) {
    inputs.statorCurrentAmps = motor.getStatorCurrent().getValueAsDouble();
    inputs.motorSpeed = motor.get();
  }

  @Override
  public void setVoltage(double voltage) {
    motor.setVoltage(voltage);
  }

  @Override
  public void stopMotor() {
    motor.set(0);
  }
}
