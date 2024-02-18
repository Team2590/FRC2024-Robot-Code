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

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.FeedForwardCharacterization;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.elevatorarm.Arm;
import frc.robot.subsystems.elevatorarm.ArmIOTalonFX;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.flywheel.FlywheelIO;
import frc.robot.subsystems.flywheel.FlywheelIOSim;
import frc.robot.subsystems.flywheel.FlywheelIOTalonFX;
import frc.robot.subsystems.user_input.UserInput;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Flywheel flywheel;
  private final Arm arm;
  private final UserInput input;

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  private final LoggedDashboardNumber flywheelSpeedInput =
      new LoggedDashboardNumber("Flywheel Speed", 1500.0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    input = UserInput.getInstance();
    switch (Constants.currentMode) {
      case REAL:
        drive =
            new Drive(
                new GyroIOPigeon2(true) {},
                new ModuleIOTalonFX(0),
                new ModuleIOTalonFX(1),
                new ModuleIOTalonFX(2),
                new ModuleIOTalonFX(3));
        flywheel = new Flywheel(new FlywheelIOTalonFX());
        // instantiate other subsystems
        arm = new Arm(new ArmIOTalonFX());

        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
        flywheel = new Flywheel(new FlywheelIOSim());
        // instantiate SIM VERSIONS F other subsystems
        arm = new Arm(new ArmIOTalonFX());
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        flywheel = new Flywheel(new FlywheelIO() {});
        arm = new Arm(new ArmIOTalonFX());
        break;
    }
    // pass in all subsystems into superstructure

    // Set up auto routines
    // autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    registerAutoCommands();
    autoChooser = new LoggedDashboardChooser<>("Auto Choices");
    populateAutoChooser();
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -input.leftJoystickY(),
            () -> -input.leftJoystickX(),
            () -> input.rightJoystickX()));
  }

  public void stop() {
    drive.stop();
    arm.stop();
  }

  public void updateSubsystems() {
    // call update functions of all subsystems
    arm.periodic();
    input.update();
  }

  public void updateUserInput() {
    // joystick inputs galore!
    // if (input.leftJoystickTriggerPressed()) {
    //   snapDrive =
    //       DriveCommands.SnapToTarget(
    //           drive, () -> input.leftJoystickY(), () -> input.leftJoystickX(), new Pose2d());
    //   CommandScheduler.getInstance().schedule(snapDrive);
    // } else if (input.leftJoystickButtonReleased(1)) {
    //   CommandScheduler.getInstance().cancel(snapDrive);
    // }
    // if (input.rightJoystickTriggerPressed()) {
    //   superstructure.intake();
    // }

    if (input.leftJoystickButtonPressed(2)) {
      arm.motionmagic1();
    } else if (input.leftJoystickButtonPressed(3)) {
      arm.motionmagicintake();
    } else if (input.leftJoystickButtonPressed(4)) {
      arm.motionmagicamp();
    } else if (input.leftJoystickButton(5)) {
      arm.armmanualup();
    } else if (input.leftJoystickButton(6)) {
      arm.armmanualdown();
    }
    // joystick inputs galore!
    // if (input.leftJoystickTrigger()) {
    // superstructure.intake();
    // superstructure.intake();
    // } else if (input.leftJoystickButton(2)) {
    // superstructure.amp();
    // } else {
    // superstructure.stop();
    // }
  }

  // --------AUTO CHOOSER FUNCTIONS------------
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
  /**
   * Use this branch to build the commands from the autos that you want to run, and add the commands
   * to the AutoChooser.
   */
  private void populateAutoChooser() {

    // Set up feedforward characterization
    autoChooser.addOption(
        "Drive FF Characterization",
        new FeedForwardCharacterization(
            drive, drive::runCharacterizationVolts, drive::getCharacterizationVelocity));
    autoChooser.addOption(
        "Flywheel FF Characterization",
        new FeedForwardCharacterization(
            flywheel, flywheel::runVolts, flywheel::getCharacterizationVelocity));
  }

  /**
   * Use this to register all of the commands with the AutoBuilder This should include all commands
   * used in the autos (except drive commands)
   */
  private void registerAutoCommands() {
    NamedCommands.registerCommand(
        "Run Flywheel",
        Commands.startEnd(
                () -> flywheel.runVelocity(flywheelSpeedInput.get()), flywheel::stop, flywheel)
            .withTimeout(5.0));
  }
}
