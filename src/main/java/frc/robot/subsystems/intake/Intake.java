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

package frc.robot.subsystems.intake;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  SimpleMotorFeedforward ffModel;
  LoggedTunableNumber intakeKs = new LoggedTunableNumber("intake/Intake/IntakeKs", 0.19578);
  LoggedTunableNumber intakeKv = new LoggedTunableNumber("Drive/Intake/IntakeKv", 0.11483);
  LoggedTunableNumber intakeKp = new LoggedTunableNumber("intake/Intake/IntakeKp");
  LoggedTunableNumber intakeKd = new LoggedTunableNumber("intake/Intake/IntakeKd");
  PIDController intakeController;
  /** Creates a new Flywheel. */
  public Intake(IntakeIO io) {
    this.io = io;

    // Switch constants based on mode (the physics simulator is treated as a
    // separate robot with different tuning)
    switch (Constants.currentMode) {
      case REAL:
        ffModel = new SimpleMotorFeedforward(0, 0);
        intakeController = new PIDController(1, 1, 1);
        break;
      case REPLAY:
        ffModel = new SimpleMotorFeedforward(0.1, 0.05);
        // io.configurePID(1.0, 0.0, 0.0); (remember to change this to set PID)
        break;
      case SIM:
        ffModel = new SimpleMotorFeedforward(0.0, 0.03);
        // io.configurePID(0.5, 0.0, 0.0);
        break;
      default:
        ffModel = new SimpleMotorFeedforward(0.0, 0.0);
        break;
    }
  }

  public void updateInputs() {
    io.updateInputs(inputs);
  }

  @Override
  public void periodic() {
    Logger.processInputs("intake/Intake", inputs);

    // Modifying PID
    if (intakeKp.hasChanged(hashCode()) || intakeKd.hasChanged(hashCode())) {
      intakeController.setPID(intakeKp.get(), 0.0, intakeKd.get());
    }
    // Modifying FeedForward
    if (intakeKs.hasChanged(hashCode()) || intakeKv.hasChanged(hashCode())) {
      ffModel = new SimpleMotorFeedforward(intakeKs.get(), intakeKv.get());
    }
  }

  /** Run open loop at the specified voltage. */
  public void runVolts(double volts) {
    io.setVoltage(volts);
  }

  /** Run closed loop at the specified velocity. */
  public void runVelocity(double velocityRPM) {
    var velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM);
    io.setVelocity(velocityRadPerSec, ffModel.calculate(velocityRadPerSec));

    // Log flywheel setpoint
    // Logger.recordOutput("Flywheel/SetpointRPM", velocityRPM);
  }

  /** Stops the flywheel. */
  public void stop() {
    io.stop();
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
}
