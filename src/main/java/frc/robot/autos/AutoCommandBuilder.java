package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Superstructure;
import frc.robot.commands.FlingCommand;
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
    if (!startPathSpecified) {
      // If the first path wasn't specified, make this the first path.
      startPath(pathName);
    } else {
      commands.addCommands(
          Commands.runOnce(() -> curr_path_name = pathName),
          Commands.print("Running FollowPathCommand for " + pathName),
          paths.getFollowPathCommand(pathName),
          Commands.print("FollowPath done:" + pathName),
          Commands.runOnce(() -> curr_path_name = "none"));
    }
    return this;
  }

  public AutoCommandBuilder fling() {

    commands.addCommands(new FlingCommand(superstructure, 1.0));
    return this;
  }

  public AutoCommandBuilder shoot() {
    commands.addCommands(
        new ShootCommand(superstructure, 1.0)); // tune the 1 second to something smaller
    return this;
  }

  public AutoCommandBuilder shoot(double setpoint) {
    commands.addCommands(
        new ShootCommand(superstructure, 1.0, setpoint)); // second param is the intake speed taken
    return this;
  }

  public static String getName() {
    return curr_path_name;
  }

  public Command build() {
    return Commands.race(Commands.run(() -> superstructure.primeShooter()), commands);
  }
}
