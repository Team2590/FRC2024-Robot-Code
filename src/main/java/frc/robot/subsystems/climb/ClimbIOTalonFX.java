package frc.robot.subsystems.climb;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants.ClimbConstants;

public class ClimbIOTalonFX implements ClimbIO {
  private final TalonFX leader = new TalonFX(24);
  private final TalonFX follower = new TalonFX(25);
  private final int tolerance = 5;

  public ClimbIOTalonFX() {
    var config = new TalonFXConfiguration();
    config.CurrentLimits.StatorCurrentLimit = 30.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    leader.getConfigurator().apply(config);
    follower.getConfigurator().apply(config);
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
    if (Math.abs(Math.abs(getRotationCount(leader)) - Math.abs(getRotationCount(follower)))
        > ClimbConstants.TOLERANCE) {
      follower.setControl(new VoltageOut(-voltage));
      leader.setControl(new VoltageOut(voltage / 2));
    } else if (Math.abs(Math.abs(getRotationCount(follower)) - Math.abs(getRotationCount(leader)))
        > ClimbConstants.TOLERANCE) {
      leader.setControl(new VoltageOut(voltage));
      follower.setControl(new VoltageOut(-voltage / 2));
    } else {
      leader.setControl(new VoltageOut(voltage));
      follower.setControl(new VoltageOut(-voltage));
    }
  }

  public void stop() {
    leader.stopMotor();
    follower.stopMotor();
  }

  public double getRotationCount() {
    return leader.getPosition().getValueAsDouble();
  }

  private double getRotationCount(TalonFX motor) {
    return motor.getPosition().getValueAsDouble();
  }

  public void resetRotationCount() {
    leader.setPosition(0);
    follower.setPosition(0);
  }
}
