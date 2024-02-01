package frc.robot.subsystems.drive;

import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.path.PathPlannerTrajectory.State;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.Supplier;

public class TrajectoryFollowerCommand extends Command {

  private final PathPlannerTrajectory trajectory;
  private final Supplier<Rotation2d> startHeading;
  private final Drive swerve;
  private final Timer timer = new Timer();
  private final LoggedTunableNumber xControllerP =
      new LoggedTunableNumber("Autos/xControllerP", 10.0);
  private final LoggedTunableNumber xControllerD =
      new LoggedTunableNumber("Autos/xControllerD", 0.0);
  private final LoggedTunableNumber yControllerP =
      new LoggedTunableNumber("Autos/yControllerP", 10.0);
  private final LoggedTunableNumber yControllerD =
      new LoggedTunableNumber("Autos/yControllerD", 0.0);
  private final LoggedTunableNumber thetaControllerP =
      new LoggedTunableNumber("Autos/thetaControllerP", 7.0);
  private final LoggedTunableNumber thetaControllerD =
      new LoggedTunableNumber("Autos/thetaControllerD", 0.0);
  private final LoggedTunableNumber maxVelocity =
      new LoggedTunableNumber("Autos/maxVel", 3.0);
  private final LoggedTunableNumber maxAcceleration =
      new LoggedTunableNumber("Autos/maxAccel", 2.0);

  private final double convergenceTime;

  public final HolonomicDriveController autonomusController =
      new HolonomicDriveController(
          new PIDController(xControllerP.get(), 0, xControllerD.get()),
          new PIDController(yControllerP.get(), 0, yControllerD.get()),
          new ProfiledPIDController(
              thetaControllerP.get(),
              0,
              thetaControllerD.get(),
              new TrapezoidProfile.Constraints(maxVelocity.get(), maxAcceleration.get())));

  public TrajectoryFollowerCommand(
      PathPlannerTrajectory trajectory,
      Supplier<Rotation2d> startHeading,
      Drive swerve,
      double convergenceTime) {
    this.trajectory = trajectory;
    this.startHeading = startHeading;
    this.swerve = swerve;
    this.convergenceTime = convergenceTime;
    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    if (startHeading != null) {
      Pose2d startPose = trajectory.getInitialTargetHolonomicPose();
      RobotContainer.poseEstimator.resetPose(startPose);
    }
    timer.reset();
    timer.start();
  }

  @Override
  public void execute() {
    State goal = (State) trajectory.sample(timer.get());
    State wpilibGoal = goal;
    Rotation2d swerveRot;
    swerveRot = goal.targetHolonomicRotation;

    Pose2d poseError =
        wpilibGoal
            .getTargetHolonomicPose()
            .relativeTo(RobotContainer.poseEstimator.getLatestPose());
    Pose2d loggedGoal = wpilibGoal.getTargetHolonomicPose();
    autonomusController.getXController().setP(xControllerP.get());
    autonomusController.getXController().setD(xControllerD.get());
    autonomusController.getYController().setP(yControllerP.get());
    autonomusController.getYController().setD(yControllerD.get());
    autonomusController.getThetaController().setP(thetaControllerP.get());
    autonomusController.getThetaController().setD(thetaControllerD.get());

    ChassisSpeeds adjustedSpeeds =
        autonomusController.calculate(
            RobotContainer.poseEstimator.getLatestPose(),
            goal.getTargetHolonomicPose(),
            goal.velocityMps,
            swerveRot);
    swerve.runVelocity(adjustedSpeeds);
  }

  @Override
  public boolean isFinished() {
    return timer.get() >= trajectory.getTotalTimeSeconds() + convergenceTime;
  }

  @Override
  public void end(boolean interrupted) {
    swerve.runVelocity(new ChassisSpeeds(0, 0, 0));
  }
}
