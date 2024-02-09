// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

/**
 * @author Dhruv and Shashank
 */
package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.util.LoggedTunableNumber;

public class ShooterTalonFX implements ShooterIO {
  private static final double GEAR_RATIO = 1;

  public static final TalonFX leader = new TalonFX(0);
  public static final TalonFX follower = new TalonFX(1);

  private final StatusSignal<Double> leaderPosition = leader.getPosition();
  private final StatusSignal<Double> leaderVelocity = leader.getVelocity();
  private final StatusSignal<Double> leaderAppliedVolts = leader.getMotorVoltage();
  private final StatusSignal<Double> leaderCurrent = leader.getStatorCurrent();

  LoggedTunableNumber kP = new LoggedTunableNumber("Shooter/kP", 0.0);
  LoggedTunableNumber kD = new LoggedTunableNumber("Shooter/kD", 0.0);
  LoggedTunableNumber kFF = new LoggedTunableNumber("Shooter/kFF", 0.0);

  private TalonFXConfiguration config;

  private double voltage = 0.0;
  private ShooterStates state;
  private double desiredRPM;

  public ShooterTalonFX() {
    config = new TalonFXConfiguration();
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.CurrentLimits.StatorCurrentLimit = 50.0;
    config.Slot0.kP = kP.get();
    config.Slot0.kD = kD.get();
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    leader.getConfigurator().apply(config);
    follower.setControl(new Follower(leader.getDeviceID(), true));
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, leaderPosition, leaderVelocity, leaderAppliedVolts, leaderCurrent);
    leader.optimizeBusUtilization();
    follower.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    updateTunableNumbers();
    BaseStatusSignal.refreshAll(leaderPosition, leaderVelocity, leaderAppliedVolts, leaderCurrent);
    inputs.appliedVolts = leader.getMotorVoltage().getValueAsDouble();
    inputs.RPM = leader.getVelocity().getValueAsDouble() * 60;
    inputs.desiredVolts = leader.getClosedLoopOutput().getValueAsDouble();
  }

  @Override
  public void setVelocity(double RPM) {
    desiredRPM = RPM;
    double velocityRadPerSec = (RPM * 2 * Math.PI) / 60;
    leader.setControl(new VelocityVoltage(velocityRadPerSec));
    state = ShooterStates.APPROACHING_SET_POINT;
  }

  @Override
  public void stop() {
    leader.stopMotor();
    state = ShooterStates.STOPPED;
  }

  public void updateTunableNumbers() {
    if (kP.hasChanged(0) || kD.hasChanged(0) || kFF.hasChanged(0)) {
      config.Slot0.kP = kP.get();
      config.Slot0.kD = kD.get();
      config.Slot0.kV = kFF.get();
      leader.getConfigurator().apply(config);
    }
  }

  public double getVoltage() {
    return voltage;
  }

  public ShooterStates getState() {
    return state;
  }

  public boolean isNearDesiredSpeed() {
    double velocityRPM = leader.getVelocity().getValueAsDouble() * 60;
    double min = leader.getClosedLoopOutput().getValueAsDouble() - 30;
    double max = leader.getClosedLoopOutput().getValueAsDouble() + 30;
    if (min < velocityRPM && velocityRPM < max) {
      return true;
    } else {
      return false;
    }
  }
}
