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

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.util.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Flywheel extends SubsystemBase {
  LoggedTunableNumber flywheelP = new LoggedTunableNumber("Flywheel/kP", 0);
  LoggedTunableNumber flywheelD = new LoggedTunableNumber("Flywheel/kD", 0);
  LoggedTunableNumber tolerance = new LoggedTunableNumber("Flywheel/toleranceRPM", 10);
  private final FlywheelIO io;
  private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();
  private final SimpleMotorFeedforward ffModel;
  private States state;
  private double currentSetpoint;
  // private final double tolerance = 10;
  public static enum States {
    STOP,
    MANUAL,
    APPROACHING_SETPOINT,
    AT_SETPOINT
  }

  /** Creates a new Flywheel. */
  public Flywheel(FlywheelIO io) {
    this.io = io;
    state = States.STOP;

    // Switch constants based on mode (the physics simulator is treated as a
    // separate robot with different tuning)
    switch (Constants.currentMode) {
      case REAL:
        ffModel = new SimpleMotorFeedforward(.17420, 0.02952);
        io.configurePID(0, 0.0, 0.0);
        break;
      case REPLAY:
        ffModel = new SimpleMotorFeedforward(0, 0.0);
        io.configurePID(0, 0.0, 0.0);
        break;
      case SIM:
        ffModel = new SimpleMotorFeedforward(0.0, 0.03);
        io.configurePID(0.5, 0.0, 0.0);
        break;
      default:
        ffModel = new SimpleMotorFeedforward(0.0, 0.0);
        break;
    }
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    updateTunableNumbers();
    Logger.processInputs("Flywheel", inputs);
    Logger.recordOutput("Flywheel/CurrentState", state);

    switch (state) {
      case MANUAL:
        // io.setSetpoint(inputs.velocity); // will change to manual output
        break;
      case STOP:
        io.stop();
        break;
      case APPROACHING_SETPOINT:
        runVelocity(currentSetpoint);
        if (Units.radiansPerSecondToRotationsPerMinute(inputs.velocityRadPerSec)
                <= currentSetpoint + tolerance.get()
            || Units.radiansPerSecondToRotationsPerMinute(inputs.velocityRadPerSec)
                >= currentSetpoint - tolerance.get()) {
          state = States.AT_SETPOINT;
        }
        break;
      case AT_SETPOINT:
        runVelocity(currentSetpoint);
        if (!(Units.radiansPerSecondToRotationsPerMinute(inputs.velocityRadPerSec)
                <= currentSetpoint + tolerance.get())
            || !(Units.radiansPerSecondToRotationsPerMinute(inputs.velocityRadPerSec)
                >= currentSetpoint - tolerance.get())) {
          state = States.APPROACHING_SETPOINT;
        }
        break;
    }
  }

  /** Run open loop at the specified voltage. */
  public void runVolts(double volts) {
    io.setVoltage(volts);
  }

  public void shoot(double rpm) {
    if (inputs.velocityRadPerSec == 0) {
      state = States.APPROACHING_SETPOINT;
    }
    currentSetpoint = rpm;
  }

  /** Run closed loop at the specified velocity. */
  public void runVelocity(double velocityRPM) {
    var velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM);
    io.setVelocity(velocityRadPerSec, ffModel.calculate(velocityRadPerSec));

    // Log flywheel setpoint
    Logger.recordOutput("Flywheel/SetpointRPM", velocityRPM);
  }

  /** Stops the flywheel. */
  public void stop() {
    io.stop();
    state = States.STOP;
    currentSetpoint = 0;
  }

  /** Returns the current velocity in RPM. */
  @AutoLogOutput
  public double getVelocityRPM() {
    return Units.radiansPerSecondToRotationsPerMinute(inputs.velocityRadPerSec);
  }

  /** Returns the current velocity in radians per second. */
  public double getCharacterizationVelocity() {
    return inputs.velocityRadPerSec;
  }

  public void updateTunableNumbers() {
    if (flywheelP.hasChanged(hashCode()) || flywheelD.hasChanged(hashCode())) {
      io.configurePID(flywheelP.get(), 0, flywheelD.get());
    }
  }
}
