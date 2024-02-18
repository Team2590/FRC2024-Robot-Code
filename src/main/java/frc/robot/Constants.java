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

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.util.LoggedTunableNumber;

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
  public static final String canbus = "Takeover";

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
    public static final double MAX_LINEAR_SPEED = Units.feetToMeters(14.5);
    public static final double TRACK_WIDTH_X = Units.inchesToMeters(18.75);
    public static final double TRACK_WIDTH_Y = Units.inchesToMeters(18.75);
    public static final double DRIVE_BASE_RADIUS =  Math.hypot(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0);
    public static final double MAX_ANGULAR_SPEED = MAX_LINEAR_SPEED / DRIVE_BASE_RADIUS;
    public static LoggedTunableNumber SNAPCONTROLLERMULTIPLIER = new LoggedTunableNumber("SnapController/MaxSpeedRatio [0,1]", .5);
    public static final Lock ODOMETRY_LOCK = new ReentrantLock();
    //module constatns below
    public static final double WHEEL_RADIUS = Units.inchesToMeters(2.0);
    public static final double ODOMETRY_FREQUENCY = 250.0;
  }

  public final class ArmConstants {
    // Fill in
  }

  public final class ShooterConstants {
  public static final double GEAR_RATIO = 1.5;
  public static final TalonFX LEADER = new TalonFX(0);
  public static final TalonFX FOLLOWER = new TalonFX(1);
  }



  public final class ConveyorConstants {
    public static final int FEEDER_ID = 0;
    public static final int DIVRETER_ID = 0;
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
    public static final int INTAKE_ID = 0;
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
