package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Superstructure;
import frc.robot.subsystems.drive.Drive;

public class AutoCommandBuilder {

  private final PathPlannerPaths paths;
  private final Drive drive;
  private final Superstructure superstructure;
  private final SequentialCommandGroup commands;
  private boolean startPathSpecified = false;

  public AutoCommandBuilder(PathPlannerPaths paths, Drive drive, Superstructure superstructure) {
    this.paths = paths;
    this.drive = drive;
    this.superstructure = superstructure;
    this.commands = new SequentialCommandGroup();
  }

  public AutoCommandBuilder startPath(String pathName) {
    startPathSpecified = true;
    commands.addCommands(
        Commands.print("Running Start Path for " + pathName),
        new StartPathCommand(paths, pathName, superstructure));
    return this;
  }

  public AutoCommandBuilder followPath(String pathName) {
    if (!startPathSpecified) {
      throw new IllegalStateException("Need to specify a startPath() before followPath()");
    }
    commands.addCommands(
        Commands.print("Running FollowPathCommand for " + pathName),
        paths.getFollowPathCommand(pathName));
    return this;
  }

  public AutoCommandBuilder intake() {
    commands.addCommands(
        Commands.print("Running intake command"),
        new InstantCommand(() -> superstructure.intake(), superstructure.getIntake()));
    return this;
  }

  public AutoCommandBuilder shoot(boolean snapToSpeaker) {
    if (snapToSpeaker) {
      // commands.addCommands(DriveCommands.SnapToTarget(drive, 0, 0, Targets.SPEAKER));
    }
    commands.addCommands(
        Commands.print("Running Shoot command"),
        new InstantCommand(() -> superstructure.shoot(), superstructure.getShooter()));
    return this;
  }

  public SequentialCommandGroup build() {
    return commands;
  }
}
