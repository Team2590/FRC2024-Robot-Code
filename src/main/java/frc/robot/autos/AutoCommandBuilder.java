package frc.robot.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.FieldConstants.Targets;
import frc.robot.RobotContainer;
import frc.robot.Superstructure;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.SnapToTargetCommand;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.GeomUtil;

public class AutoCommandBuilder {

  private final PathPlannerPaths paths;
  private final Drive drive;
  private final Superstructure superstructure;
  private final SequentialCommandGroup commands;
  private boolean isPoseReset = false;

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
    commands.addCommands(
        Commands.runOnce(
            () -> {
              Pose2d translatedPose = GeomUtil.flipPoseBasedOnAlliance(pose);
              RobotContainer.poseEstimator.resetPose(translatedPose);
            }));
    isPoseReset = true;
    return this;
  }

  public AutoCommandBuilder followPath(String pathName) {
    if (!isPoseReset) {
      // If the pose wasn't reset, use the first path to reset.
      resetPoseUsingPath(pathName);
    }
    commands.addCommands(
        Commands.print("Running FollowPathCommand for " + pathName),
        paths.getFollowPathCommand(pathName));
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
      commands.addCommands(new SnapToTargetCommand(drive, () -> 0, () -> 0, Targets.SPEAKER, .05));
    }

    commands.addCommands(new ShootCommand(superstructure, 3));
    return this;
  }

  public Command build() {
    return Commands.race(Commands.run(() -> superstructure.primeShooter()), commands);
  }
}
