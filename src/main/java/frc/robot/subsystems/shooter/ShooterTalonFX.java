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

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.util.LoggedTunableNumber;

public class ShooterTalonFX implements ShooterIO {
  private static final double GEAR_RATIO = 1;

  public static final TalonFX leader = new TalonFX(69);
  public static final TalonFX follower = new TalonFX(42);

  private final StatusSignal<Double> leaderPosition = leader.getPosition();
  private final StatusSignal<Double> leaderVelocity = leader.getVelocity();
  private final StatusSignal<Double> leaderAppliedVolts = leader.getMotorVoltage();
  private final StatusSignal<Double> leaderCurrent = leader.getStatorCurrent();

  LoggedTunableNumber kP = new LoggedTunableNumber("Shooter/kP", .4);
  LoggedTunableNumber kD = new LoggedTunableNumber("Shooter/kD", 0);
  LoggedTunableNumber kFF = new LoggedTunableNumber("Shooter/kFF", .2);

  private PIDController PID = new PIDController(kP.get(), 0, kD.get());
  private TalonFXConfiguration config;

  private double voltage = 0.0;

  public ShooterTalonFX() {
    config = new TalonFXConfiguration();
    config.CurrentLimits.StatorCurrentLimit = 50.0;
    config.Slot0.kP = kP.get();
    config.Slot0.kD = kD.get();
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    leader.getConfigurator().apply(config);
    follower.getConfigurator().apply(config);
    follower.setControl(new Follower(leader.getDeviceID(), false));
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, leaderPosition, leaderVelocity, leaderAppliedVolts, leaderCurrent);
    leader.optimizeBusUtilization();
    follower.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    updateTunableNumbers();
    BaseStatusSignal.refreshAll(leaderPosition, leaderVelocity, leaderAppliedVolts, leaderCurrent);
    // inputs.velocityRadPerSec =
    //     Units.rotationsToRadians(leaderVelocity.getValueAsDouble()) / GEAR_RATIO;
    // inputs.appliedVolts = leaderAppliedVolts.getValueAsDouble();
    inputs.appliedVolts = this.voltage;
  }

  @Override
  public void setVoltage(double voltage) {
    this.voltage = voltage;
    leader.setVoltage(voltage);
  }

  @Override
  public void setVelocity(double RPM) {
    double velocityRadPerSec = (2 * Math.PI) / 60;
    System.out.println(velocityRadPerSec);
    double voltage = PID.calculate(velocityRadPerSec, leader.get()) + kFF.get();
    setVoltage(voltage);
  }

  @Override
  public void stop() {
    leader.stopMotor();
    setVoltage(0);
    System.out.println(leader.getMotorVoltage());
  }

  public void updateTunableNumbers() {
    if (kP.hasChanged(0) || kD.hasChanged(0)) {
      config.Slot0.kP = kP.get();
      config.Slot0.kD = kD.get();
      leader.getConfigurator().apply(config);
      follower.getConfigurator().apply(config);
      PID.setPID(kP.get(), 0, kD.get());
    }
  }

  public double getVoltage() {
    return this.voltage;
  }
}
