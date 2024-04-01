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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AprilTag;
import frc.robot.util.GeomUtil;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class DriveCommands {
  private static final double DEADBAND = 0.1;

  private DriveCommands() {}

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   */
  public static Command joystickDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    return Commands.run(
        () -> {
          // Apply deadband
          double linearMagnitude =
              MathUtil.applyDeadband(
                  Math.hypot(xSupplier.getAsDouble(), ySupplier.getAsDouble()), DEADBAND);
          Rotation2d linearDirection =
              new Rotation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble());
          double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

          // Square values
          linearMagnitude = linearMagnitude * linearMagnitude;
          omega = Math.copySign(omega * omega, omega);

          // Calcaulate new linear velocity
          Translation2d linearVelocity =
              new Pose2d(new Translation2d(), linearDirection)
                  .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                  .getTranslation();

          // Convert to field relative speeds & send command
          drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                  linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                  omega * drive.getMaxAngularSpeedRadPerSec(),
                  drive.getGyroYaw()));
        },
        drive);
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
    return Commands.run(() -> snapToTargetForAuto(drive, xSupplier, ySupplier, target), drive);
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
          // Logger.recordOutput(
          //     "Drive/NoteController/PID Output",
          //     drive.noteController.calculate(-yError.getAsDouble(), 0)
          //         * drive.getMaxLinearSpeedMetersPerSec());
          // Logger.recordOutput("Drive/NoteController/YError", yError.getAsDouble());
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

  public static Command alignClimb(
      Drive drive, DoubleSupplier xSupplier, int aprilTag, double lateralOffset) {
    double angleSetpoint;
    switch (aprilTag) {
      case (12):
        angleSetpoint = 240;
        break;
      case (15):
        angleSetpoint = 120;
        break;
      case (11):
        angleSetpoint = 120;
        break;
      case (16):
        angleSetpoint = 240;
        break;
      case (13):
        angleSetpoint = 0;
        break;
      case (14):
        angleSetpoint = 0;
        break;
      default:
        angleSetpoint = 0;
    }

    return Commands.run(
        () -> {
          double theta = drive.getGyroYaw().getDegrees() % 360;
          double currentError = theta - angleSetpoint;

          if (currentError > 180) {
            currentError -= 360;
          } else if (currentError < -180) {
            currentError += 360;
          }

          Logger.recordOutput("Odometry/current theta in stage", theta);

          drive.runVelocity(
              new ChassisSpeeds(
                  xSupplier.getAsDouble()
                      * drive.getMaxLinearSpeedMetersPerSec()
                      * .25, // Adjusted linear speed
                  drive.linearMovementController.calculate(lateralOffset, 0)
                      * drive.getMaxLinearSpeedMetersPerSec(), // Lateral movement
                  drive.snapController.calculate(currentError, 0)
                      * .25 // Adjusted angular speed for shortest rotation path
                  ));
        },
        drive);
  }

  public static void snapToTargetForAuto(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      FieldConstants.Targets target) {
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
                    AprilTag.getTagPose(14), AprilTag.getTagPose(15), AprilTag.getTagPose(16));
        break;
      case FLING:
        targetPose =
            DriverStation.getAlliance().get() == Alliance.Blue
                ? Constants.FlingConstants.BLUE_FLING_POSE
                : Constants.FlingConstants.RED_FLING_POSE;
        break;
      default:
        targetPose = new Pose2d();
        break;
    }
    // find angle
    Transform2d difference = RobotContainer.poseEstimator.getLatestPose().minus(targetPose);
    // double angleOffset = DriverStation.getAlliance().get() == Alliance.Red ? Math.PI : 0;
    double theta = Math.atan2(difference.getY(), difference.getX());
    double currentAngle = drive.getGyroYaw().getRadians() % (2 * Math.PI); // CHANGED
    // Logger.recordOutput("SnapController/RealCurrentAngle",
    // drive.getGyroYaw().getRadians());
    // Logger.recordOutput("SnapController/CurrentAngle", currentAngle);
    double currentError = theta - currentAngle;
    if (currentError > Math.PI) {
      currentAngle += 2 * Math.PI;
    } else if (currentError < -Math.PI) {
      currentAngle -= 2 * Math.PI;
    }
    // Logger.recordOutput("SnapController/CurrentError", currentError);
    // Logger.recordOutput("SnapController/Target", target);
    // Logger.recordOutput("SnapController/TargetPose", targetPose);
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
            drive.getGyroYaw()));
  }
}
