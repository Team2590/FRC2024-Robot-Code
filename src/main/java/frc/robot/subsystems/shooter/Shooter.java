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
 *     <p>*
 */
package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.HashMap;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private ShooterTalonFX shooterMotors = new ShooterTalonFX();
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
  HashMap<Double, Double> table = new HashMap<Double, Double>();

  public Shooter() {
    table.put(10.0, 1000.0);
    table.put(20.0, 2000.0);
    table.put(30.0, 3000.0);
  }

  @Override
  public void periodic() {
    shooterMotors.updateTunableNumbers();
    shooterMotors.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
    System.out.println(shooterMotors.getVoltage());
  }

  public void shoot(double distance) {
    System.out.println("Shooting");
    if (table.containsKey(distance)) {
      double rpm = table.get(distance);
      shooterMotors.setVelocity(rpm);
    } else {
      double rpm = 0.0; // need to write logic to get the rpm using the linear regression
      shooterMotors.setVelocity(rpm);
    }
  }

  public void stop() {
    shooterMotors.stop();
  }
}
