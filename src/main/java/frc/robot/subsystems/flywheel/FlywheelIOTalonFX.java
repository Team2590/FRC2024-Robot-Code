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
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.util.Units;

public class FlywheelIOTalonFX implements FlywheelIO {
  private static final double GEAR_RATIO = 1.5;

  private final TalonFX leader = new TalonFX(0);
  private final TalonFX follower = new TalonFX(1);
  private final double controllerTolerance = 0.2;
  private double controllerSetpoint = 100 / 60;
  private StatusSignal<Double> velocity;
  private final BangBangController controller;

  private final StatusSignal<Double> leaderPosition = leader.getPosition();
  private final StatusSignal<Double> leaderVelocity = leader.getVelocity();
  private final StatusSignal<Double> leaderAppliedVolts = leader.getMotorVoltage();
  private final StatusSignal<Double> leaderCurrent = leader.getStatorCurrent();
  private final StatusSignal<Double> followerCurrent = follower.getStatorCurrent();

  public FlywheelIOTalonFX() {
    var config = new TalonFXConfiguration();
    config.CurrentLimits.StatorCurrentLimit = 30.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    leader.getConfigurator().apply(config);
    follower.getConfigurator().apply(config);
    follower.setControl(new Follower(leader.getDeviceID(), false));

    velocity = leader.getVelocity();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        leaderPosition,
        leaderVelocity,
        leaderAppliedVolts,
        leaderCurrent,
        followerCurrent,
        velocity);
    leader.optimizeBusUtilization();
    follower.optimizeBusUtilization();
    controller = new BangBangController(controllerTolerance);
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        leaderPosition,
        leaderVelocity,
        leaderAppliedVolts,
        leaderCurrent,
        followerCurrent,
        velocity);
    inputs.positionRad = Units.rotationsToRadians(leaderPosition.getValueAsDouble()) / GEAR_RATIO;
    inputs.velocityRadPerSec =
        Units.rotationsToRadians(leaderVelocity.getValueAsDouble()) / GEAR_RATIO;
    inputs.appliedVolts = leaderAppliedVolts.getValueAsDouble();
    inputs.currentAmps =
        new double[] {leaderCurrent.getValueAsDouble(), followerCurrent.getValueAsDouble()};
    inputs.velocity = (leader.getVelocity().getValueAsDouble()) / 60;
  }

  @Override
  public void setVoltage(double volts) {
    leader.setControl(new VoltageOut(volts));
  }

  @Override
  public double returnVelocity() {
    return leader.getVelocity().getValueAsDouble();
  }

  public boolean atSetpoint() {
    if ((returnVelocity() >= controllerSetpoint - controllerTolerance)
        && (returnVelocity() <= controllerSetpoint + controllerTolerance)) {
      return true;
    } else {
      return false;
    }
  }

  public void setSetpoint(double setpointFR) {
    controllerSetpoint = setpointFR;
  }

  @Override
  /** Run the flywheel at the predetemined setpoint */
  public void runVelocity() {
    leader.set(controller.calculate(returnVelocity(), controllerSetpoint));
    follower.set(-1 * (controller.calculate(returnVelocity(), controllerSetpoint)));

    // leader.setControl(
    //     new VelocityVoltage(
    //         Units.radiansToRotations(velocityRadPerSec),
    //         0.0,
    //         true,
    //         ffVolts,
    //         0,
    //         false,
    //         false,
    //         false));
    // leader.set(.4);

  }

  @Override
  public void stop() {
    leader.stopMotor();
  }

  // @Override
  // public void configurePID(double kP, double kI, double kD) {
  //   var config = new Slot0Configs();
  //   config.kP = kP;
  //   config.kI = kI;
  //   config.kD = kD;
  //   leader.getConfigurator().apply(config);
  // }
}
