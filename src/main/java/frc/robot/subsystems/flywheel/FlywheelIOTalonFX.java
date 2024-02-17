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

package frc.robot.subsystems.flywheel;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.FlywheelIOInputs;;


public class FlywheelIOTalonFX implements FlywheelIO {




  private final StatusSignal<Double> leaderPosition = ShooterConstants.LEADER.getPosition();
  private final StatusSignal<Double> leaderVelocity = ShooterConstants.LEADER.getVelocity();
  private final StatusSignal<Double> leaderAppliedVolts = ShooterConstants.LEADER.getMotorVoltage();
  private final StatusSignal<Double> leaderCurrent = ShooterConstants.LEADER.getStatorCurrent();
  private final StatusSignal<Double> followerCurrent = ShooterConstants.FOLLOWER.getStatorCurrent();

  public FlywheelIOTalonFX() {
    var config = new TalonFXConfiguration();
    config.CurrentLimits.StatorCurrentLimit = 30.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    ShooterConstants.LEADER.getConfigurator().apply(config);
    ShooterConstants.FOLLOWER.getConfigurator().apply(config);
    ShooterConstants.FOLLOWER.setControl(new Follower(ShooterConstants.LEADER.getDeviceID(), true));

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, leaderPosition, leaderVelocity, leaderAppliedVolts, leaderCurrent, followerCurrent);
    ShooterConstants.LEADER.optimizeBusUtilization();
    ShooterConstants.FOLLOWER.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        leaderPosition, leaderVelocity, leaderAppliedVolts, leaderCurrent, followerCurrent);
    FlywheelIOInputs.inputs.POSITIONRAD = Units.rotationsToRadians(leaderPosition.getValueAsDouble()) / ShooterConstants.GEAR_RATIO;
    FlywheelIOInputs.inputs.VELOCITYRADPERSEC =
        Units.rotationsToRadians(leaderVelocity.getValueAsDouble()) / ShooterConstants.GEAR_RATIO;
    FlywheelIOInputs.inputs.APPLIEDVOLTS = leaderAppliedVolts.getValueAsDouble();
    FlywheelIOInputs.inputs.CURRENTAMPS =
        new double[] {leaderCurrent.getValueAsDouble(), followerCurrent.getValueAsDouble()};
  }

  @Override
  public void setVoltage(double volts) {
    ShooterConstants.LEADER.setControl(new VoltageOut(volts));
  }

  @Override
  public void setVelocity(double velocityRadPerSec, double ffVolts) {
    ShooterConstants.LEADER.setControl(
        new VelocityVoltage(
            Units.radiansToRotations(velocityRadPerSec),
            0.0,
            true,
            ffVolts,
            0,
            false,
            false,
            false));
  }

  @Override
  public void stop() {
    ShooterConstants.LEADER.stopMotor();
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    var config = new Slot0Configs();
    config.kP = kP;
    config.kI = kI;
    config.kD = kD;
    ShooterConstants.LEADER.getConfigurator().apply(config);
  }
}
