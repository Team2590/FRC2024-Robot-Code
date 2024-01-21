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

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LinearRegression;
import java.util.ArrayList;
import java.util.HashMap;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private ShooterTalonFX shooterMotors = new ShooterTalonFX();
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
  HashMap<Double, Double> table = new HashMap<Double, Double>(100);

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
      System.out.println("RPM: " + rpm);
      shooterMotors.setVelocity(rpm);
    } else {
      double[] distances = getNearestDistances(distance);
      double[] rpms = {table.get(distances[0]), table.get(distances[1])};
      LinearRegression regression = new LinearRegression(distances, rpms);
      double rpm = regression.slope() * distance + regression.intercept();
      System.out.println("RPM: " + rpm);
      shooterMotors.setVelocity(rpm);
    }
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
    double[] distances = {result.get(result.size() - 2), result.get(result.size() - 1)};
    return distances;
  }

  public void setRPM(double RPM) {
    shooterMotors.setVelocity(RPM);
  }

  public void stop() {
    shooterMotors.stop();
  }
}
