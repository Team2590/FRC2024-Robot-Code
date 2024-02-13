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
import edu.wpi.first.units.Units;
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
  LoggedTunableNumber kV = new LoggedTunableNumber("Shooter/kV", 0.0);
  LoggedTunableNumber kS = new LoggedTunableNumber("Shooter/kS", 0.0);

  private TalonFXConfiguration config;

  private ShooterStates state;

  public ShooterTalonFX() {
    config = new TalonFXConfiguration();
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.CurrentLimits.StatorCurrentLimit = 50.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.Slot0.kP = kP.get();
    config.Slot0.kD = kD.get();
    config.Slot0.kV = kV.get();
    config.Slot0.kS = kS.get();
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

  /**
   * @param velocity in RPM
   */
  @Override
  public void setVelocity(double RPM) {
    leader.setControl(new VelocityVoltage(RPM / 60).withSlot(0));
    state = ShooterStates.APPROACHING_SET_POINT;
  }

  @Override
  public void stop() {
    leader.stopMotor();
    state = ShooterStates.STOPPED;
  }

  public void updateTunableNumbers() {
    if (kP.hasChanged(0) || kD.hasChanged(0) || kV.hasChanged(0) || kS.hasChanged(0)) {
      config.Slot0.kP = kP.get();
      config.Slot0.kD = kD.get();
      config.Slot0.kV = kV.get();
      config.Slot0.kS = kS.get();
      leader.getConfigurator().apply(config);
    }
  }

  public void setVoltage(double voltage) {
    leader.setVoltage(voltage);
  }

  public double getVoltage() {
    return leader.getMotorVoltage().getValueAsDouble();
  }

  public double getRPM() {
    return leader.getVelocity().getValueAsDouble() * 60;
  }

  public double getAngularVelocity() {
    return Units.RadiansPerSecond.convertFrom(
        leader.getVelocity().getValue(), Units.RotationsPerSecond);
  }

  public double getAngularPosition() {
    return Units.Radians.convertFrom(leader.getPosition().getValue(), Units.Rotations);
  }

  public double getPosition() {
    return leader.getPosition().getValueAsDouble();
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
