package frc.robot.subsystems.climb;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants.ClimbConstants;
import frc.robot.util.LoggedTunableNumber;

import org.littletonrobotics.junction.Logger;

public class ClimbIOTalonFX implements ClimbIO {
  Slot0Configs slot0;
  TalonFXConfiguration config;
  private final TalonFX leader = new TalonFX(24);
  private final TalonFX follower = new TalonFX(25);
  private final int tolerance = 5;
  private final PositionVoltage m_voltagePosition = new PositionVoltage(0, 0, true, 0, 0, false, false, false);
  LoggedTunableNumber kP = new LoggedTunableNumber("Climb/kP", 0.1);

  public ClimbIOTalonFX() {
    config = new TalonFXConfiguration();
    config.CurrentLimits.StatorCurrentLimit = 30.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.Slot0.kP = .1;
    leader.getConfigurator().apply(config);
    follower.getConfigurator().apply(config);
    follower.setControl(new Follower(leader.getDeviceID(), true));

    leader.setPosition(0);
  }

  @Override
  public void updateInputs(ClimbIOInputs inputs) {
    inputs.leaderPositionRotations = leader.getPosition().getValueAsDouble();
    inputs.followerPositionRotations = follower.getPosition().getValueAsDouble();
    Logger.recordOutput("Climb/LeaderPosition", getLeaderRotationCount());
    Logger.recordOutput("Climb/FollowerPosition", getFollowerRotationCount());
  }

  @Override
  public void run(double speed) {
    leader.set(-speed);
    follower.set(-speed);
  }

  @Override
  public void setVoltage(double voltage) {
    if (Math.abs(getRotationCount(leader)) - Math.abs(getRotationCount(follower))
        > ClimbConstants.TOLERANCE) {
      follower.setControl(new VoltageOut(-voltage));
      leader.setControl(new VoltageOut(voltage / 4));
    } else if (Math.abs(getRotationCount(follower)) - Math.abs(getRotationCount(leader))
        > ClimbConstants.TOLERANCE) {
      leader.setControl(new VoltageOut(voltage));
      follower.setControl(new VoltageOut(-voltage / 4));
    } else {
      leader.setControl(new VoltageOut(voltage));
      follower.setControl(new VoltageOut(-voltage));
    }
  }

  public void stop() {
    leader.stopMotor();
    follower.stopMotor();
  }

  public double getLeaderRotationCount() {
    return leader.getPosition().getValueAsDouble();
  }

  public double getFollowerRotationCount() {
    return follower.getPosition().getValueAsDouble();
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

  @Override
  public void stopLeader() {
    leader.stopMotor();
  }

  @Override
  public void stopFollower() {
    follower.stopMotor();
  }

  @Override
  public void setPosition(double position){
    leader.setControl(m_voltagePosition.withPosition(position));
  }

  public void updateTunableNumbers() {
    if (kP.hasChanged(0)) {
      slot0.kP = kP.get();
      leader.getConfigurator().apply(config);
    }
  }
}
