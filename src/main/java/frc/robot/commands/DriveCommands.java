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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.controller.PIDController;

public class DriveCommands {
  private static final double DEADBAND = 0.1;
  private static final PIDController noteController = new PIDController(0,0,0); // temp

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
                  drive.getRotation()));
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
   * @see <a href =
   *     "https://github.com/Team254/FRC-2022-Public/blob/main/src/main/java/com/team254/lib/control/SwerveHeadingController.java">Code
   *     Reference</a>
   */
  public static Command SnapToTarget(
      Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier, Pose2d target) {
    return Commands.run(
        (() -> {
          Transform2d difference = drive.getPose().minus(target);
          double theta = Math.atan2(difference.getY(), difference.getX());
          double currentAngle = drive.getRotation().getRadians();
          double currentError = theta - currentAngle;
          if (currentError > Math.PI) {
            currentAngle += 2 * Math.PI;
          } else if (currentError < -Math.PI) {
            currentAngle -= 2 * Math.PI;
          }
          Logger.recordOutput("SnapController/TargetPose", target);
          drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  -xSupplier.getAsDouble()
                      * drive.getMaxLinearSpeedMetersPerSec()
                      * Drive.snapControllermultiplier.get(),
                  -ySupplier.getAsDouble()
                      * drive.getMaxLinearSpeedMetersPerSec()
                      * Drive.snapControllermultiplier.get(),
                  drive.snapController.calculate(currentAngle, theta)
                      * drive.getMaxAngularSpeedRadPerSec(),
                  drive.getRotation()));
        }),
        drive);
  }
  
  /**
   * Translate in line to a grounded note. Use camera data to get relative note pose.
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
          drive.runVelocity(
            new ChassisSpeeds(
              // joystick magnitude
              Math.hypot(xSupplier.getAsDouble(),ySupplier.getAsDouble())
                * drive.getMaxLinearSpeedMetersPerSec()
                * Drive.snapControllermultiplier.get(),
              noteController.calculate(yError.getAsDouble(), 0)
                * drive.getMaxLinearSpeedMetersPerSec()
                * Drive.snapControllermultiplier.get(),
              0
            ));
        }),
        drive);
  }
}
