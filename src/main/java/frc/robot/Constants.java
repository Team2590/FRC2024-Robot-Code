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

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
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

  public static final double CAMERA_HEIGHT_METERS = 45.5 / 100;

  public static final double CAMERA_X_DISTANCE_FROM_CENTER_METERS = Units.inchesToMeters(12.5);
  public static final double CAMERA_Y_DISTANCE_FROM_CENTER_METERS = Units.inchesToMeters(.5);
  public static final double CAMERA_ROLL = 0;
  public static final double CAMERA_PITCH = Units.degreesToRadians(45);
  public static final double CAMERA_YAW = 0;
  public static final double roll = Constants.CAMERA_ROLL;
  public static final double pitch = Constants.CAMERA_PITCH;
  public static final double yaw = Constants.CAMERA_YAW;
  public static final Transform3d RobotToCam =
      new Transform3d(
          -1 * CAMERA_X_DISTANCE_FROM_CENTER_METERS,
          -1 * CAMERA_Y_DISTANCE_FROM_CENTER_METERS,
          -1 * CAMERA_HEIGHT_METERS,
          new Rotation3d(roll, pitch, yaw));
  // public static final Transform3d camPose = new
  // Transform3d(CAMERA_X_DISTANCE_FROM_CENTER_CENTIMETERS,
  //
  // CAMERA_Y_DISTANCE_FROM_CENTER_CENTIMETERS,
  //                                                           CAMERA_HEIGHT_METERS,
  //                                                           new Rotation3d());

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }
}