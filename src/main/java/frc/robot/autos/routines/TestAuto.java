package frc.robot.autos.routines;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.TrajectoryFollowerCommand;
import java.util.List;

public class TestAuto extends SequentialCommandGroup {

  public TestAuto(Drive drive) {
    addRequirements(drive);

    Pose2d startPos = new Pose2d(new Translation2d(2.0, 5.0), new Rotation2d());
    Pose2d endPos = new Pose2d(new Translation2d(5.0, 5.0), new Rotation2d());
    List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(startPos, endPos);
    PathPlannerPath path =
        new PathPlannerPath(
            bezierPoints,
            new PathConstraints(4.0, 4.0, Units.degreesToRadians(360), Units.degreesToRadians(540)),
            new GoalEndState(0.0, new Rotation2d()));

    PathPlannerTrajectory traj = path.getTrajectory(new ChassisSpeeds(), new Rotation2d());
    addCommands(
        new WaitCommand(2),
        new TrajectoryFollowerCommand(
            traj, () -> traj.getInitialTargetHolonomicPose().getRotation(), drive, 5),
        new WaitCommand(10));
  }
}
