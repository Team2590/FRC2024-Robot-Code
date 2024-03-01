package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.util.GeomUtil;
import org.littletonrobotics.junction.Logger;

/** Command to Initialize starting pose. */
public class ResetPoseCommand extends Command {

  private final Pose2d initialPose;

  public ResetPoseCommand(Pose2d initialPose) {
    this.initialPose = initialPose;
  }

  @Override
  public void execute() {
    Pose2d translatedPose = GeomUtil.flipPoseBasedOnAlliance(initialPose);
    RobotContainer.poseEstimator.resetPose(translatedPose);
    Logger.recordOutput("Auto/Trace", "Resetting Pose");
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
