package frc.robot.util;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import java.util.ArrayList;

public class FlyPathBuilder {
  private ArrayList<Pose2d> positions = new ArrayList<Pose2d>();
  private GoalEndState endState;
  private PathConstraints constraints;

  public PathPlannerPath build() {
    var converted = PathPlannerPath.bezierFromPoses(positions);
    return new PathPlannerPath(converted, constraints, endState);
  }

  public FlyPathBuilder addPose(Pose2d pose) {
    positions.add(pose);
    return this;
  }

  public FlyPathBuilder addPoses(Pose2d... poses) {
    for (var pose : poses) positions.add(pose);
    return this;
  }

  public FlyPathBuilder addConstraints(PathConstraints constraints) {
    this.constraints = constraints;
    return this;
  }

  public FlyPathBuilder addGoalEndState(GoalEndState endState) {
    this.endState = endState;
    return this;
  }
}
