package frc.robot.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.GeometryUtil;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.GeometryUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.Superstructure;

/**
 * Performs all the initialization needed and executes the first path.
 *
 * <p>This should be the first path in the auto routine.
 */
public class StartPathCommand extends SequentialCommandGroup {

  public StartPathCommand(
      PathPlannerPaths paths, String startingPath, Superstructure superstructure) {
    Pose2d startingPose = paths.getStartingPose(startingPath);
    Pose2d translated_pose =
        DriverStation.getAlliance().isPresent()
                && (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue)
            ? startingPose
            : GeometryUtil.flipFieldPose(startingPose);
    addCommands(
        // Initialize the starting pose based on the start path.
        new InstantCommand(() -> RobotContainer.poseEstimator.resetPose(translated_pose)),
        // Start up the intake system and follow path to first position in parallel.
        Commands.parallel(
            Commands.print("Starting up Intake .... "),
            new InstantCommand(() -> superstructure.intake()),
            // Move to note1 from starting position B (speaker)
            AutoBuilder.followPath(startingPath)));
  }
}
