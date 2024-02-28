package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.FieldConstants.Targets;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AprilTag;
import frc.robot.util.GeomUtil;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

/**
 * Snaps to target command.
 *
 * <p>The command keeps executing until one of the following conditions is true:
 *
 * <p>- the currentError is within errorTolerance - OR till some timeout (2.0 seconds)
 */
public class SnapToTargetCommand extends Command {

  private static final double WAIT_TIME_SECONDS = 2.0;

  private final Drive drive;
  private final DoubleSupplier xSupplier;
  private final DoubleSupplier ySupplier;
  private final Targets target;
  private final double errorTolerance;
  private final Timer timer = new Timer();

  private Pose2d targetPose;
  private double currentError;
  private int count;

  public SnapToTargetCommand(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      FieldConstants.Targets target,
      double errorTolerance) {
    addRequirements(drive);
    this.drive = drive;
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    this.target = target;
    this.errorTolerance = errorTolerance;
  }

  @Override
  public void initialize() {
    count = 0;
    timer.restart();
    boolean isRed = DriverStation.getAlliance().get() == Alliance.Red;
    switch (target) {
      case SPEAKER:
        targetPose = isRed ? AprilTag.getTagPose(4) : AprilTag.getTagPose(7);
        break;
      case AMP:
        targetPose = isRed ? AprilTag.getTagPose(5) : AprilTag.getTagPose(6);
        break;
      case STAGE:
        targetPose =
            isRed
                ? GeomUtil.triangleCenter(
                    AprilTag.getTagPose(11), AprilTag.getTagPose(12), AprilTag.getTagPose(13))
                : GeomUtil.triangleCenter(
                    AprilTag.getTagPose(14), AprilTag.getTagPose(15), AprilTag.getTagPose(16));
        break;
      default:
        targetPose = new Pose2d();
        break;
    }
  }

  @Override
  public void execute() {
    count++;
    // find angle
    count++;
    Transform2d difference = RobotContainer.poseEstimator.getLatestPose().minus(targetPose);
    double angleOffset = DriverStation.getAlliance().get() == Alliance.Red ? Math.PI : 0;
    double theta = Math.atan2(difference.getY(), difference.getX()) + angleOffset;
    double currentAngle = RobotContainer.poseEstimator.getLatestPose().getRotation().getRadians();
    currentError = theta - currentAngle;
    if (currentError > Math.PI) {
      currentAngle += 2 * Math.PI;
    } else if (currentError < -Math.PI) {
      currentAngle -= 2 * Math.PI;
    }
    Logger.recordOutput("SnapController/Error", currentError);
    Logger.recordOutput("SnapController/Target", target);
    Logger.recordOutput("SnapController/TargetPose", targetPose);

    System.out.printf(
        "----> SnapToTargetComand: currentError[%f], currentAngle[%f], currentAngleDegree[%f]\n",
        currentError, currentAngle, Units.radiansToDegrees(currentAngle));
    // run the motors
    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            xSupplier.getAsDouble()
                * drive.getMaxLinearSpeedMetersPerSec()
                * Drive.snapControllermultiplier.get(),
            ySupplier.getAsDouble()
                * drive.getMaxLinearSpeedMetersPerSec()
                * Drive.snapControllermultiplier.get(),
            drive.snapController.calculate(currentAngle, theta)
                * drive.getMaxAngularSpeedRadPerSec(),
            RobotContainer.poseEstimator.getLatestPose().getRotation()));
  }

  @Override
  public boolean isFinished() {
    // Wait a few seconds or if we are within error tolerance to stop running this command.
    return timer.hasElapsed(WAIT_TIME_SECONDS) || count > 25;
    // return false;
  }

  @Override
  public void end(boolean interrupted) {
    timer.stop();
    drive.stop(); // added dt stop
  }
}