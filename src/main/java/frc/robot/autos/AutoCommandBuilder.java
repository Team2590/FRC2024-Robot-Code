package frc.robot.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.FieldConstants.Targets;
import frc.robot.Superstructure;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ResetPoseCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.SnapToTargetCommand;
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.Logger;

public class AutoCommandBuilder {

  private final PathPlannerPaths paths;
  private final Drive drive;
  private final Superstructure superstructure;
  private final SequentialCommandGroup commands;

  public AutoCommandBuilder(PathPlannerPaths paths, Drive drive, Superstructure superstructure) {
    this.paths = paths;
    this.drive = drive;
    this.superstructure = superstructure;
    this.commands = new SequentialCommandGroup();
  }

  public AutoCommandBuilder resetPoseUsingPath(String pathName) {
    Pose2d poseFromPath = paths.getStartingPose(pathName);
    return resetPose(poseFromPath);
  }

  public AutoCommandBuilder resetPose(Pose2d pose) {
    commands.addCommands(new ResetPoseCommand(pose));
    return this;
  }

  public AutoCommandBuilder followPath(String pathName) {
    commands.addCommands(
        trace("FollowPath:" + pathName),
        Commands.parallel(
            paths.getFollowPathCommand(pathName),
            Commands.either(
                trace("Note Present, not starting Intake"),
                new IntakeCommand(superstructure, 1.0),
                () -> superstructure.note_present())));
    return this;
  }

  public AutoCommandBuilder intake() {
    commands.addCommands(new IntakeCommand(superstructure, 1.0));
    return this;
  }

  public AutoCommandBuilder shoot(boolean snapToSpeaker) {
    if (snapToSpeaker) {
      commands.addCommands(new SnapToTargetCommand(drive, () -> 0, () -> 0, Targets.SPEAKER, 2.0));
    }
    commands.addCommands(new ShootCommand(superstructure, 1.0));
    return this;
  }

  public SequentialCommandGroup build() {
    return commands;
  }

  private static final Command trace(String message) {
    return Commands.runOnce(() -> Logger.recordOutput("Auto/Trace", message));
  }
}
