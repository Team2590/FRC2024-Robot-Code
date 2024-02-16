// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

// import subsystems
import frc.robot.subsystems.conveyor.*;
import frc.robot.subsystems.intake.Intake;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class Superstructure {
  // TBD: declare variables to add subsystems into
  private static enum States {
    DISABLED,
    INTAKE,
    SHOOT,
    CLIMB,
    AMP,
    RESET
  }

  private States state = States.DISABLED;
  private Conveyor conveyor;
  private Intake intake;
  // private final LoggedDashboardNumber flywheelSpeedInput =
  //     new LoggedDashboardNumber("Flywheel Speed", 1500.0);

  /** The container for the robot. Pass in the appropriate subsystems from RobotContainer */
  public Superstructure(Conveyor conveyor, Intake intake) {
    // assign args to local variables
    this.conveyor = conveyor;
    this.intake = intake;
  }

  /** Call all of the periodic methods of the subsystems */
  public void updateSubsystems() {}

  /** This is where you would call all of the periodic functions of the subsystems. */
  public void periodic() {
    switch (state) {
      case DISABLED:
        // stop
        conveyor.setStopped();
        break;
      case INTAKE:
        // intaking
        // conveyor.setIntaking();
        intake.setIntake();
        break;
      case SHOOT:
        // shooting
        conveyor.setShooting();
        break;
      case CLIMB:
        // climbing
        conveyor.setStopped();
        break;
      case RESET:
        // return all subsystems to its home state
        conveyor.setStopped();
        break;
      case AMP:
        // amp
        conveyor.setDiverting();
        break;
    }
  }

  public void intake() {
    state = States.INTAKE;
  }

  public void reset() {
    state = States.RESET;
  }

  public void shoot() {
    state = States.SHOOT;
  }

  public void climb() {
    state = States.CLIMB;
  }

  public void stop() {
    state = States.DISABLED;
  }

  public void amp() {
    state = States.AMP;
  }
}
