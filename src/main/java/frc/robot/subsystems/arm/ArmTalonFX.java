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
package frc.robot.subsystems.arm;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.util.LoggedTunableNumber;

public class ArmTalonFX implements ArmIO {
  private static final double GEAR_RATIO = 1;

  public static final TalonFX leader = new TalonFX(21, "Jazzy");

  private final StatusSignal<Double> leaderPosition = leader.getPosition();
  private final StatusSignal<Double> leaderVelocity = leader.getVelocity();
  private final StatusSignal<Double> leaderAppliedVolts = leader.getMotorVoltage();
  private final StatusSignal<Double> leaderCurrent = leader.getStatorCurrent();

  LoggedTunableNumber kP = new LoggedTunableNumber("Shooter/kP", 0);
  LoggedTunableNumber kD = new LoggedTunableNumber("Shooter/kD", 0);
  LoggedTunableNumber kFF = new LoggedTunableNumber("Shooter/kFF", 0);

  private TalonFXConfiguration config;

  private CANcoder encoder = new CANcoder(0);

  private double position = 0.0;

  public ArmTalonFX() {
    config = new TalonFXConfiguration();
    config.CurrentLimits.StatorCurrentLimit = 50.0;
    config.Slot0.kP = kP.get();
    config.Slot0.kD = kD.get();
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    leader.getConfigurator().apply(config);
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, leaderPosition, leaderVelocity, leaderAppliedVolts, leaderCurrent);
    leader.optimizeBusUtilization();
    encoder.setControl(new Follower(leader.getDeviceID(), false));
    encoder.setPosition(0);
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    updateTunableNumbers();
    BaseStatusSignal.refreshAll(leaderPosition, leaderVelocity, leaderAppliedVolts, leaderCurrent);
    inputs.appliedPosition = leader.getPosition().getValueAsDouble();
    inputs.desiredPosition = this.position;
    inputs.absolutePosition = encoder.getAbsolutePosition().getValueAsDouble();
    inputs.appliedVolts = leader.getMotorVoltage().getValueAsDouble();
  }

  @Override
  public void setPosition(double position) {
    System.out.println("Set position called");
    leader.setControl(new PositionVoltage(position));
  }

  @Override
  public void stop() {
    leader.stopMotor();
  }

  public void updateTunableNumbers() {
    if (kP.hasChanged(0) || kD.hasChanged(0)) {
      config.Slot0.kP = kP.get();
      config.Slot0.kD = kD.get();
      leader.getConfigurator().apply(config);
    }
  }

  public double getPosition() {
    return this.position;
  }
}
