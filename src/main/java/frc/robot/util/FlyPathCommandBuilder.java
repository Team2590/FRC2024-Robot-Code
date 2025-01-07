package frc.robot.util;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

public class FlyPathCommandBuilder {
  private Pose2d targetPose;
  private PathConstraints constraints;

  public Command build() {
    return AutoBuilder.pathfindToPose(targetPose, constraints, 0);
  }

  public FlyPathCommandBuilder addConstraints(PathConstraints _constraints) {
    constraints = _constraints;
    return this;
  }

  public FlyPathCommandBuilder addTargetPose(Pose2d pose) {
    targetPose = pose;
    return this;
  }
}
