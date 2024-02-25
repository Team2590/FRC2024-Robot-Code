package frc.robot.subsystems.climb;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ClimbIOTalonFX implements ClimbIO {
  private final TalonFX leader = new TalonFX(24, "Takeover");
  private final TalonFX follower = new TalonFX(25, "Takeover");

  public ClimbIOTalonFX() {
    var config = new TalonFXConfiguration();
    config.CurrentLimits.StatorCurrentLimit = 30.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    leader.getConfigurator().apply(config);
    follower.getConfigurator().apply(config);
    // follower.setControl(new Follower(leader.getDeviceID(), true));
  }

  @Override
  public void updateInputs(ClimbIOInputs inputs) {
    inputs.positionRotations = getRotationCount();
  }

  @Override
  public void run(double speed) {
    leader.set(-speed);
    follower.set(-speed);
  }

  @Override
  public void setVoltage(double voltage) {
    leader.setControl(new VoltageOut(voltage));
    follower.setControl(new VoltageOut(-voltage));
  }

  public void stop() {
    leader.stopMotor();
    follower.stopMotor();
  }

  public double getRotationCount() {
    return leader.getPosition().getValueAsDouble();
  }

  public void resetRotationCount() {
    leader.setPosition(0);
    follower.setPosition(0);
  }
}
