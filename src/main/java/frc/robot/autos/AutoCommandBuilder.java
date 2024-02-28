package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.FieldConstants.Targets;
import frc.robot.Superstructure;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ShootCommand;
import frc.robot.subsystems.drive.Drive;

public class AutoCommandBuilder {

  private final PathPlannerPaths paths;
  private final Drive drive;
  private final Superstructure superstructure;
  private final SequentialCommandGroup commands;
  private boolean startPathSpecified = false;

  private static String curr_path_name = "none";

  public AutoCommandBuilder(PathPlannerPaths paths, Drive drive, Superstructure superstructure) {
    this.paths = paths;
    this.drive = drive;
    this.superstructure = superstructure;
    this.commands = new SequentialCommandGroup();
  }

  public AutoCommandBuilder startPath(String pathName) {
    curr_path_name = pathName;
    startPathSpecified = true;
    commands.addCommands(
        Commands.print("Running Start Path for " + pathName),
        new StartPathCommand(paths, pathName, superstructure));
    return this;
  }

  public AutoCommandBuilder followPath(String pathName) {
    curr_path_name = pathName;
    if (!startPathSpecified) {
      // If the first path wasn't specified, make this the first path.
      startPath(pathName);
    } else {
      commands.addCommands(
          Commands.print("Running FollowPathCommand for " + pathName),
          Commands.parallel(
              paths.getFollowPathCommand(pathName),
              Commands.run(() -> superstructure.intake()).until(superstructure::note_present)));
    }
    return this;
  }

  public AutoCommandBuilder intake() {
    commands.addCommands(
        Commands.print("Running intake command"),
        new InstantCommand(() -> superstructure.intake()));

    return this;
  }

  public AutoCommandBuilder shoot(boolean snapToSpeaker) {
    if (snapToSpeaker) {
      commands.addCommands(
          Commands.race(
              DriveCommands.SnapToTarget(drive, () -> 0, () -> 0, Targets.SPEAKER),
              Commands.waitUntil(() -> drive.snapController.atSetpoint())));

      // TODO-should
    }

    commands.addCommands(new ShootCommand(superstructure, 3));
    return this;
  }

  public static String getName() {
    return curr_path_name;
  }
  // TODO Add a method to keep the shooter primed while moving.

  public SequentialCommandGroup build() {
    return commands;
  }
}
