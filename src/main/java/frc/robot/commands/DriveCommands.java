// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.FieldConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AprilTag;
import frc.robot.util.GeomUtil;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public interface DriveCommands {

  public static final double DEADBAND = 0.1;

  /**
   * Maps an input from -1 to 1 to an output from -1 to 1.
   *
   * @param x - [-1, 1]
   * @return - [-1, 1]
   */
  private static double scale(double x) {
    return Math.copySign(x * x, x);
  }

  /**
   * Field relative drive command using one joystick (controlling linear). Desmos:
   * https://www.desmos.com/calculator/cswpncuxr2
   *
   * @param drive - the drive subsystem
   * @param xSupplier - function to supply x values [-1, 1]
   * @param ySupplier - function to supply y values [-1, 1]
   * @param angularVelocitySupplier - function to supply the angular velocity for the chassis speeds
   * @return the drive command
   */
  public static Command oneJoystickDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier angularVelocitySupplier) {
    return Commands.run(
        () -> {
          final var x = xSupplier.getAsDouble();
          final var y = ySupplier.getAsDouble();

          final var magnitude = Math.hypot(x, y);

          if (magnitude < DEADBAND) {
            drive.runVelocity(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    0, 0, angularVelocitySupplier.getAsDouble(), drive.getRotation()));
            return;
          }

          final var newMagnitude =
              scale((magnitude - Math.copySign(DEADBAND, magnitude)) / (1 - DEADBAND));
          final var newX =
              newMagnitude > 1
                  ? Math.signum(x) / Math.hypot(y / x, 1)
                  : newMagnitude * x / magnitude;
          final var newY =
              newMagnitude > 1
                  ? Math.signum(y) / Math.hypot(x / y, 1)
                  : newMagnitude * y / magnitude;

          drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  newX * Drive.MAX_LINEAR_SPEED,
                  newY * Drive.MAX_LINEAR_SPEED,
                  angularVelocitySupplier.getAsDouble(),
                  drive.getRotation()));
        },
        drive);
  }

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   *
   * @param drive - the drive subsystem
   * @param xSupplier - function to supply x values [-1, 1]
   * @param ySupplier - function to supply y values [-1, 1]
   * @param omegaSupplier - function to supply omega (angular) values [-1, 1]
   * @return the drive command
   */
  public static Command joystickDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    return oneJoystickDrive(
        drive,
        xSupplier,
        ySupplier,
        () ->
            scale(MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND))
                * Drive.MAX_ANGULAR_SPEED);
  }

  /**
   * Snap to the target position, while maintaining joystick movement
   *
   * @param drive - drive instance
   * @param xSupplier - left joystick x value
   * @param ySupplier - left joystick y value
   * @param target - pos to snap to
   * @return the command
   * @author Ian Keller
   * @see <a href =
   *     "https://github.com/Team254/FRC-2022-Public/blob/main/src/main/java/com/team254/lib/control/SwerveHeadingController.java">Code
   *     Reference</a>
   */
  public static Command SnapToTarget(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      FieldConstants.Targets target) {
    return Commands.run(
        (() -> {
          // get target pose
          Pose2d targetPose;
          switch (target) {
            case SPEAKER:
              targetPose =
                  DriverStation.getAlliance().get() == Alliance.Red
                      ? AprilTag.getTagPose(4)
                      : AprilTag.getTagPose(7);
              break;
            case AMP:
              targetPose =
                  DriverStation.getAlliance().get() == Alliance.Red
                      ? AprilTag.getTagPose(5)
                      : AprilTag.getTagPose(6);
              break;
            case STAGE:
              targetPose =
                  DriverStation.getAlliance().get() == Alliance.Red
                      ? GeomUtil.triangleCenter(
                          AprilTag.getTagPose(11), AprilTag.getTagPose(12), AprilTag.getTagPose(13))
                      : GeomUtil.triangleCenter(
                          AprilTag.getTagPose(14),
                          AprilTag.getTagPose(15),
                          AprilTag.getTagPose(16));
              break;
            default:
              targetPose = new Pose2d();
              break;
          }
          // find angle
          Transform2d difference = RobotContainer.poseEstimator.getLatestPose().minus(targetPose);
          double angleOffset = DriverStation.getAlliance().get() == Alliance.Red ? Math.PI : 0;
          double theta = Math.atan2(difference.getY(), difference.getX()) + angleOffset;
          double currentAngle =
              RobotContainer.poseEstimator.getLatestPose().getRotation().getRadians();
          double currentError = theta - currentAngle;
          if (currentError > Math.PI) {
            currentAngle += 2 * Math.PI;
          } else if (currentError < -Math.PI) {
            currentAngle -= 2 * Math.PI;
          }
          Logger.recordOutput("SnapController/Target", target);
          Logger.recordOutput("SnapController/TargetPose", targetPose);
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
        }),
        drive);
  }

  /**
   * Translate in line to a grounded note. Use camera data to get relative note pose.
   *
   * @param drive - drive instance
   * @param xSupplier - left joystick x value
   * @param ySupplier - left joystick y value
   * @param yError - lateral error from note; take directly from note camera
   * @return the command
   */
  public static Command translateToNote(
      Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier yError) {
    return Commands.run(
        (() -> {
          Logger.recordOutput(
              "Drive/NoteController/PID Output",
              drive.noteController.calculate(-yError.getAsDouble(), 0)
                  * drive.getMaxLinearSpeedMetersPerSec());
          Logger.recordOutput("Drive/NoteController/YError", yError.getAsDouble());
          drive.runVelocity(
              new ChassisSpeeds(
                  // joystick magnitude
                  -Math.hypot(xSupplier.getAsDouble(), ySupplier.getAsDouble())
                      * drive.getMaxLinearSpeedMetersPerSec(),
                  drive.noteController.calculate(-yError.getAsDouble(), 0)
                      * drive.getMaxLinearSpeedMetersPerSec()
                      * Drive.noteControllermultiplier.get(),
                  0));
        }),
        drive);
  }

  public static Command turnToNote(
      Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier yawSupplier) {
    final var yaw = yawSupplier.getAsDouble();
    return joystickDrive(
        drive,
        xSupplier,
        ySupplier,
        () -> {
          return Math.abs(yaw) <= 1 ? 0 : -yaw / 50 - Math.signum(yaw) / 10;
        });
  }
}
