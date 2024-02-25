package frc.robot.subsystems.climb;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ClimbIOTalonFX implements ClimbIO {
  private final TalonFX left_motor = new TalonFX(24, "Takeover");
  private final TalonFX right_motor = new TalonFX(25, "Takeover");

  public ClimbIOTalonFX() {
    left_motor.setNeutralMode(NeutralModeValue.Brake);
    right_motor.setNeutralMode(NeutralModeValue.Brake);
    right_motor.setControl(new Follower(left_motor.getDeviceID(), true));
  }

  @Override
  public void updateInputs(ClimbIOInputs inputs) {
    inputs.positionRotations = getRotationCount();
  }

  @Override
  public void run(double speed) {
    left_motor.set(-speed);
    right_motor.set(-speed);
  }

  @Override
  public void setVoltage(double voltage) { 
    left_motor.setVoltage(-voltage);
  }

  public void stop() {
    left_motor.stopMotor();
    // right_motor.stopMotor();
  }

  public double getRotationCount() {
    return left_motor.getPosition().getValueAsDouble();
  }

  public void resetRotationCount() {
    left_motor.setPosition(0);
    right_motor.setPosition(0);
  }
}
