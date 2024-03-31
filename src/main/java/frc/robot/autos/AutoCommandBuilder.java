package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.FieldConstants.Targets;
import frc.robot.Superstructure;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.SnapToTargetCommand;
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
    // curr_path_name = pathName;

    if (!startPathSpecified) {
      // If the first path wasn't specified, make this the first path.
      startPath(pathName);
    } else {
      commands.addCommands(
          Commands.runOnce(
              () -> {
                curr_path_name = pathName;
              }),
          Commands.print("Running FollowPathCommand for " + pathName),
          paths.getFollowPathCommand(pathName),
          Commands.print("FollowPath done:" + pathName));
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
    // if (snapToSpeaker) {
    //   // commands.addCommands(
    //   //     Commands.parallel(
    //   //         new SnapToTargetCommand(
    //   //             drive,
    //   //             () -> 0,
    //   //             () -> 0,
    //   //             Targets.SPEAKER,
    //   //             .5d // TODO: Figure out the best error tolerance.
    //   //             ),
    //   //         new ShootCommand(superstructure, 0.5)));
    //   // Commands.race(
    //   //     DriveCommands.SnapToTarget(drive, () -> 0, () -> 0, Targets.SPEAKER),
    //   //     Commands.waitSeconds(2.0)));
    // } else {
    commands.addCommands(
        new ShootCommand(superstructure, 1)); // tune the 1 second to something smaller
    // }
    return this;
  }

  public AutoCommandBuilder shoot(boolean snapToSpeaker, int setpoint) {
    if (snapToSpeaker) {
      commands.addCommands(
          new SnapToTargetCommand(
              drive,
              () -> 0,
              () -> 0,
              Targets.SPEAKER,
              0.5d // TODO: Figure out the best error tolerance.
              ));
      // Commands.race(
      //     DriveCommands.SnapToTarget(drive, () -> 0, () -> 0, Targets.SPEAKER),
      //     Commands.waitSeconds(2.0)));
    }

    commands.addCommands(
        new ShootCommand(superstructure, .5, setpoint)); // second param is the intake speed taken

    return this;
  }

  public static String getName() {
    return curr_path_name;
  }

  public Command build() {
    return Commands.race(Commands.run(() -> superstructure.primeShooter()), commands);
  }
}
