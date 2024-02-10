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

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final Mode currentMode = Mode.REAL;
  public static final boolean tuningMode = true;

  public final class FieldConstants {
    public static final double FIELD_LENGTH_METERS = 16.54175;
    public static final double FIELD_WIDTH_METERS = 8.0137;
    public static final Pose2d FLIPPING_POSE =
        new Pose2d(
            new Translation2d(FIELD_LENGTH_METERS, FIELD_WIDTH_METERS), new Rotation2d(Math.PI));
  }

  public final class VisionConstants {
    public static final double CAMERA_HEIGHT_METERS = 45.5 / 100;
    public static final double CAMERA_X_DISTANCE_FROM_CENTER_METERS = Units.inchesToMeters(12.5);
    public static final double CAMERA_Y_DISTANCE_FROM_CENTER_METERS = Units.inchesToMeters(.5);
    public static final double CAMERA_ROLL = 0;
    public static final double CAMERA_PITCH = Units.degreesToRadians(45);
    public static final double CAMERA_YAW = 0;
    public static final double APRILTAG_AMBIGUITY_THRESHOLD = 0.2;
    public static final Transform3d RobotToCam =
        new Transform3d(
            -1 * CAMERA_X_DISTANCE_FROM_CENTER_METERS,
            -1 * CAMERA_Y_DISTANCE_FROM_CENTER_METERS,
            -1 * CAMERA_HEIGHT_METERS,
            new Rotation3d(CAMERA_ROLL, CAMERA_PITCH, CAMERA_YAW));
  }

  public final class DrivetrainConstants {
    // Fill in
  }

  public final class ArmConstants {
    // Fill in
  }

  public final class ShooterConstants {
    // Fill in
  }

  public final class ConveyorConstants {
    // Fill in
  }

  public final class IntakeConstants {
    // Fill in
  }

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY, 

    /** Running Kang */
    KANG,

    /** Running Jynx */
    JYNX
  }
}
