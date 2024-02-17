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

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.FeedForwardCharacterization;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.flywheel.FlywheelIO;
import frc.robot.subsystems.flywheel.FlywheelIOSim;
import frc.robot.subsystems.flywheel.FlywheelIOTalonFX;
import frc.robot.util.PoseEstimator;
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
  public static final PoseEstimator poseEstimator =
      new PoseEstimator(VecBuilder.fill(0.003, 0.003, 0.0002));

  // Controller
  private final CommandJoystick left_Joystick = new CommandJoystick(0);
  private final CommandJoystick right_Joystick = new CommandJoystick(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  private final LoggedDashboardNumber flywheelSpeedInput =
      new LoggedDashboardNumber("Flywheel Speed", 1500.0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOTalonFX(0),
                new ModuleIOTalonFX(1),
                new ModuleIOTalonFX(2),
                new ModuleIOTalonFX(3));
        flywheel = new Flywheel(new FlywheelIOTalonFX());
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
        break;
    }

    // Set up auto routines
    NamedCommands.registerCommand(
        "Closed Loop Shoot",
        Commands.startEnd(() -> flywheel.runVelocity(1.18), flywheel::stop, flywheel)
            .withTimeout(5));

    NamedCommands.registerCommand(
        "Open Loop Shoot",
        Commands.startEnd(() -> flywheel.runVolts(.5), flywheel::stop, flywheel).withTimeout(3));

    NamedCommands.registerCommand("print Command", flywheel.printCom());
    // autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    // SendableChooser<Command> sendable = new SendableChooser<>();
    // sendable.addOption("demo3piece", AutoBuilder.buildAuto("demo3piece"));
   
 

    NamedCommands.registerCommand("print Command", flywheel.printCom());
    // autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    // SendableChooser<Command> sendable = new SendableChooser<>();
    // sendable.addOption("demo3piece", AutoBuilder.buildAuto("demo3piece"));
    autoChooser = new LoggedDashboardChooser<Command>("Auto Choice");

    // Set up feedforward characterization
    //  autoChooser.addOption("demo3piece", AutoBuilder.buildAuto("demo3piece"));
    // autoChooser.addOption("Under the Stage 3 piece",
    // AutoBuilder.buildAuto("under_stage_3piece"));
    autoChooser.addOption("inception_test", AutoBuilder.buildAuto("testing"));
    autoChooser.addOption(
        "3 piece root amp sweep", AutoBuilder.buildAuto("3 piece root amp sweep"));
    autoChooser.addOption("for_you", AutoBuilder.buildAuto("heulitt_auto"));

    // autoChooser.addOption(
    // "3 piece root far sweep", AutoBuilder.buildAuto("3 piece root far sweep"));

    // autoChooser.addOption("test_auto", AutoBuilder.buildAuto("testing"));

    autoChooser.addOption(
        "Drive FF Characterization",
        new FeedForwardCharacterization(
            drive, drive::runCharacterizationVolts, drive::getCharacterizationVelocity));
    autoChooser.addOption(
        "Flywheel FF Characterization",
        new FeedForwardCharacterization(
            flywheel, flywheel::runVolts, flywheel::getCharacterizationVelocity));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -left_Joystick.getY(),
            () -> -left_Joystick.getX(),
            () -> right_Joystick.getX()));
    right_Joystick.button(2).onTrue(Commands.runOnce(drive::stopWithX, drive));
    right_Joystick
        .button(3)
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));
    right_Joystick
        .button(1)
        .whileTrue(Commands.startEnd(() -> flywheel.runVolts(.3), flywheel::stop, flywheel));
    right_Joystick.button(4).onTrue(Commands.runOnce(drive::zeroGyro, drive));
  }

  // public void initRobot(String name) {

  //   drive.setGyro(PathPlannerAuto.getStaringPoseFromAutoFile(name).getRotation().getDegrees());
  //   //

  // }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
