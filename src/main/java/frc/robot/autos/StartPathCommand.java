package frc.robot.autos;

import edu.wpi.first.math.geometry.Pose2d;
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
    addCommands(
        // Initialize the starting pose based on the start path.
        new InstantCommand(
            () -> {
              Pose2d translatedPose = GeomUtil.flipPoseBasedOnAlliance(startingPose);
              RobotContainer.poseEstimator.resetPose(translatedPose);
              // RobotContainer.getDrive().gyroIO.setGyro(translatedPose.getRotation().getDegrees());
            }));
  }
}
