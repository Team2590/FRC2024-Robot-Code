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

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LinearRegression;
import java.util.ArrayList;
import java.util.HashMap;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private ShooterTalonFX shooterMotors = new ShooterTalonFX();
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
  HashMap<Double, Double> table = new HashMap<Double, Double>(100);

  private ShooterStates state;

  public Shooter() {
    table.put(10.0, 1000.0);
    table.put(20.0, 2000.0);
    table.put(30.0, 3000.0);
    table.put(40.0, 4000.0);
    table.put(50.0, 5000.0);
    table.put(60.0, 6000.0);
    table.put(70.0, 7000.0);
  }

  @Override
  public void periodic() {
    shooterMotors.updateTunableNumbers();
    shooterMotors.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
  }

  public void shoot(double distance) {
    if (table.containsKey(distance)) {
      double rpm = table.get(distance);
      System.out.println("Computed Shooter RPM: " + rpm);
      shooterMotors.setVelocity(rpm);
    } else {
      double rpm = getComptuedRPM(distance);
      System.out.println("Computed Shooter RPM: " + rpm);
      shooterMotors.setVelocity(rpm);
    }
  }

  private double getComptuedRPM(double distance) {
    double[] distances = getNearestDistances(distance);
    double[] rpms = { table.get(distances[0]), table.get(distances[1]) };
    LinearRegression regression = new LinearRegression(distances, rpms);
    double rpm = regression.slope() * distance + regression.intercept();
    return rpm;
  }

  private double[] getNearestDistances(double actualDistance) {
    var result = new ArrayList<Double>();
    for (Double distance : table.keySet()) {
      if (distance > actualDistance) {
        result.add(distance);
        break;
      } else if (distance < actualDistance) {
        result.add(distance);
      }
    }
    double[] distances = { result.get(result.size() - 2), result.get(result.size() - 1) };
    return distances;
  }

  public void setRPM(double rpm) {
    System.out.println(rpm);
    shooterMotors.setVelocity(rpm);
  }

  public void setVoltage(double voltage) {
    shooterMotors.setVoltage(voltage);
  }

  public void setVoltage(Measure<Voltage> voltage) {
    // shooterMotors.setVoltage(voltage.in(Units.Volts));
    shooterMotors.setVoltage(voltage.baseUnitMagnitude());
  }

  public void stop() {
    shooterMotors.stop();
  }

  public void setManualControl() {
    state = ShooterStates.MANUAL;
  }

  public void removeManualControl() {
    state = shooterMotors.getState();
  }

  // public void sysIdLog(SysIdRoutineLog logger) {
  // logger.motor("Leader").voltage(Units.Volts.of(shooterMotors.getVoltage()));
  // logger
  // .motor("Leader")
  // .angularVelocity(Units.RadiansPerSecond.of(shooterMotors.getAngularVelocity()));
  // logger.motor("Leader").angularPosition(Units.Radians.of(shooterMotors.getAngularPosition()));
  // }

  /**
   * @return Returns the current state of the shooter subsystem
   */
  public ShooterStates getState() {
    if (state == ShooterStates.MANUAL) {
      return state;
    } else if (state == ShooterStates.APPROACHING_SET_POINT) {
      if (shooterMotors.isNearDesiredSpeed()) {
        return ShooterStates.AT_SET_POINT;
      } else {
        return ShooterStates.APPROACHING_SET_POINT;
      }
    } else if (state == ShooterStates.STOPPED) {
      return ShooterStates.STOPPED;
    }
    return null;
  }

  public void setState(ShooterStates newState) {
    state = newState;
  }
}
