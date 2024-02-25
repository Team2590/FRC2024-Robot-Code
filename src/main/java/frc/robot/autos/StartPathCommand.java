package frc.robot.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.Superstructure;
import frc.robot.util.GeomUtil;

/**
 * Performs all the initialization needed and executes the first path.
 *
 * <p>This should be the first path in the auto routine.
 */
public class StartPathCommand extends SequentialCommandGroup {

  public StartPathCommand(
      PathPlannerPaths paths, String startingPath, Superstructure superstructure) {
    Pose2d startingPose = paths.getStartingPose(startingPath);
    Pose2d translatedPose = GeomUtil.flipPoseBasedOnAlliance(startingPose);
    addCommands(
        // Initialize the starting pose based on the start path.
        new InstantCommand(() -> RobotContainer.poseEstimator.resetPose(translatedPose)),
        // Start up the intake system and follow path to first position in parallel.
        Commands.parallel(
            Commands.print("Starting up Intake .... "),
            // Starting Running the shooter
            new InstantCommand(() -> superstructure.primeShooter(), superstructure.getShooter()),
            new InstantCommand(() -> superstructure.intake(), superstructure.getIntake()),
            paths.getFollowPathCommand(startingPath)));
  }
}
