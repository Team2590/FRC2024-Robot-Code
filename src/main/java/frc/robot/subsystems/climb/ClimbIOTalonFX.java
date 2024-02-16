package frc.robot.subsystems.climb;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ClimbIOTalonFX implements ClimbIO {
  private final TalonFX left_motor = new TalonFX(1);
  private final TalonFX right_motor = new TalonFX(0);

  public final double maxPosition = 5;

  private final double speed = 0.15;

  public ClimbIOTalonFX() {
    left_motor.setNeutralMode(NeutralModeValue.Brake);
    right_motor.setNeutralMode(NeutralModeValue.Brake);
    left_motor.setInverted(true);
  }

  @Override
  public void updateInputs(ClimbIOInputs inputs) {
    inputs.positionRotations = left_motor.getPosition().getValueAsDouble();
  }

  public void up() {
    if (left_motor.getPosition().getValueAsDouble() > maxPosition) {
      left_motor.stopMotor();
      right_motor.stopMotor();
    } else {
      left_motor.set(-speed);
      right_motor.set(speed);
    }
  }

  public void down() {
    if (left_motor.getPosition().getValueAsDouble() < 0) {
      left_motor.stopMotor();
      right_motor.stopMotor();
    } else {
      left_motor.set(speed);
      right_motor.set(-speed);
    }
  }

  public void stop() {
    left_motor.stopMotor();
    right_motor.stopMotor();
  }

  public void resetRotationCount() {
    left_motor.setPosition(0);
    right_motor.setPosition(0);
  }
}
