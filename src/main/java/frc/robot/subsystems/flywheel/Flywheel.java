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

import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.Logger;

public class Flywheel extends SubsystemBase {
  private final FlywheelIO io;
  private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();
  private States states;
  public double velocity;
  private boolean shooting;

  public static enum States {
    STOP,
    MANUAL,
    APPROACHINGSETPOINT,
    ATSETPOINT
  }

  /** Creates a new Flywheel. */
  public Flywheel(FlywheelIO io) {
    this.io = io;
    states = States.STOP;
  }

  @Override
  public void periodic() {
    // io.updateInputs(inputs);
    Logger.processInputs("Flywheel", inputs);

    switch (states) {
      case MANUAL:
        io.setSetpoint(inputs.velocity); // will change to manual output
        System.out.println("In Manual");
        break;
      case STOP:
        io.stop();
        // System.out.println("Stopped");
        break;
      case APPROACHINGSETPOINT:
        io.runVelocity();
        System.out.println("Approaching Setpoint");
        break;
      case ATSETPOINT:
        io.runVelocity();
        // System.out.println("At Setpoint");
        break;
    }
  }

  public void atSetpointRun() {
    states = States.ATSETPOINT;
    System.out.println("Running");
  }

  public void approachSetpoint() {
    states = States.APPROACHINGSETPOINT;
  }

  public void shoot() {
    // boolean x = false;
    // while(!io.atSetpoint()){
    //   if(x == false){
    //   states = States.APPROACHINGSETPOINT;
    //   }
    //   else{
    //     break;
    //   }
    // }
    // if(io.atSetpoint()){
    //   states = States.ATSETPOINT;
    //   x = true;
    // }

    if (!io.atSetpoint() && !shooting) {
      states = States.APPROACHINGSETPOINT;
    } else {
      shooting = true;
      states = States.STOP;
    }
  }

  public void stop() {
    shooting = false;
    states = States.STOP;
  }

  /** Run open loop at the specified voltage. */
  public void runVolts(double volts) {
    io.setVoltage(volts);
  }

  /**
   * Run closed loop at the specified velocity. *
   *
   * <p>/** Returns the current velocity in RPM.
   */
}
