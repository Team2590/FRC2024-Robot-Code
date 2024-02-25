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

import com.ctre.phoenix6.signals.InvertedValue;
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
  public static final String CANBUS = "Takeover";

  public final class FieldConstants {
    public static final double FIELD_LENGTH_METERS = 16.54175;
    public static final double FIELD_WIDTH_METERS = 8.0137;
    public static final Pose2d FLIPPING_POSE =
        new Pose2d(
            new Translation2d(FIELD_LENGTH_METERS, FIELD_WIDTH_METERS), new Rotation2d(Math.PI));

    public static enum Targets {
      SPEAKER,
      AMP,
      STAGE
    }
  }

  public final class VisionConstants {
    public static final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(18.75);
    public static final double CAMERA_X_DISTANCE_FROM_CENTER_METERS = Units.inchesToMeters(10.948);
    public static final double CAMERA_Y_DISTANCE_FROM_CENTER_METERS = Units.inchesToMeters(0);
    public static final double CAMERA_ROLL = 0;
    // downward pitch is positive
    public static final double CAMERA_PITCH = Units.degreesToRadians(-1 * 27);
    // counterclockwise yaw is positive
    public static final double CAMERA_YAW = Units.degreesToRadians(-1 * 180);
    public static final double APRILTAG_AMBIGUITY_THRESHOLD = 0.2;
    public static final Transform3d RobotToCam =
        new Transform3d(
            -CAMERA_X_DISTANCE_FROM_CENTER_METERS,
            CAMERA_Y_DISTANCE_FROM_CENTER_METERS,
            CAMERA_HEIGHT_METERS,
            new Rotation3d(CAMERA_ROLL, CAMERA_PITCH, CAMERA_YAW));
  }

  public final class DrivetrainConstants {
    // Fill in
  }

  public final class ArmConstants {
    // Fill in
    public static final double HOME_SETPOINT = 0.168;
    public static final int ARM = 45;
    public static final int ARM_CANCODER_ID = 44;
    public static final double ARM_GEAR_RATIO = 266.67;
    public static final double MAG_OFFSET = -.156;
  }

  public final class ShooterConstants {
    // Fill in
    public static final int LEADER = 55;
    public static final int FOLLOWER = 56;
  }

  public final class ConveyorConstants {
    public static final int FEEDER_ID = 57;
    public static final int DIVRETER_ID = 58;
    public static final int INTAKE_PROX_ID = 0;
    public static final int SHOOTER_PROX_ID = 1;
    public static final double SHOOTER_PROX_THRESHOLD = 0.55; // value was tested for
    public static final double INTAKE_PROX_THRESHOLD = 0.55; // value was tested for

    public static final double DIVERTER_GEAR_RATIO = 1;
    public static final double FEEDER_GEAR_RATIO = 1;
    public static final InvertedValue diverterDirection = InvertedValue.CounterClockwise_Positive;
    public static final InvertedValue feederDirection = InvertedValue.Clockwise_Positive;
  }

  public final class IntakeConstants {
    // Fill in
    public static final int INTAKE_ID = 14;
  }

  public final class ClimbConstants {
    // max rotations = (distance/2pi*wheelRadius) * gearRatio
    public static final double MAX_ROTATIONS = -150;
  }

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }
}
