
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

//import subsystems

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class Superstructure{
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
  // private final LoggedDashboardNumber flywheelSpeedInput =
  //     new LoggedDashboardNumber("Flywheel Speed", 1500.0);

  /** The container for the robot. Pass in the appropriate subsystems from RobotContainer */
  public Superstructure() {
    // assign args to local variables
  }

  /** Call all of the periodic methods of the subsystems */
  public void updateSubsystems() {}

  /** This is where you would call all of the periodic functions of the subsystems. */
  public void periodic() {
    switch (state) {
      case DISABLED:
        // stop
        break;
      case INTAKE:
        // intaking
        break;
      case SHOOT:
        // shooting
        break;
      case CLIMB:
        // climbing
        break;
      case RESET:
        // return all subsystems to its home state
        break;
      case AMP:
        //amp
        break;
    }
  }
  public void intake(){
    state = States.INTAKE;
  }
  public void reset(){
    state = States.RESET;
  }
  public void shoot(){
    state = States.SHOOT;
  }
  public void climb(){
    state = States.CLIMB;
  }
  public void stop() {
    state = States.DISABLED;
  }
  public void amp(){
    state = States.AMP; 
  }
}
