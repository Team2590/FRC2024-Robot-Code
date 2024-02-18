package frc.robot.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;

/**
 * Performs all the initialization needed and executes the first path.
 *
 * This should be the first path in the auto routine.
 */
public class StartPathCommand extends SequentialCommandGroup {

  public StartPathCommand(PathPlannerPaths paths, String startingPath) {
    Pose2d startingPose = paths.getStartingPose(startingPath);
    addCommands(
        // Initialize the starting pose based on the start path.
        new InstantCommand(() -> RobotContainer.poseEstimator.resetPose(startingPose)),
        // Start up the intake system and follow path to first position in parallel.
        Commands.parallel(
            // TODO: command to start the start the intake system.
            Commands.print("Starting up Intake .... "),
            // Move to note1 from starting position B (speaker)
            paths.getFollowPathCommand(startingPath)));
  }

}
