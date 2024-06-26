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
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Constants.LEDConstants.Colors;
import frc.robot.util.LoggedTunableNumber;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : Mode.SIM;
  public static final boolean tuningMode = true;
  public static final String CANBUS = "Takeover";
  public static LoggedTunableNumber ampTuned = new LoggedTunableNumber("Arm/AMP Setpoint", -0.27);
  public static LoggedTunableNumber homeSetpoint =
      new LoggedTunableNumber("Arm/IntakeSetpoint", .155);

  public static LoggedTunableNumber trapTuning =
      new LoggedTunableNumber("Arm/TRAP Setpoint", -0.32);

  public final class FieldConstants {
    // Error Tolerance for Snapping.
    public static final double SNAP_ERROR_TOLERANCE = 0.05;
    public static final double RUMBLE_THRESHOLD = 6;
    public static final double FIELD_LENGTH_METERS = 16.54175;
    public static final double FIELD_WIDTH_METERS = 8.0137;
    public static final Pose2d FLIPPING_POSE =
        new Pose2d(
            new Translation2d(FIELD_LENGTH_METERS, FIELD_WIDTH_METERS), new Rotation2d(Math.PI));

    public static enum Targets {
      SPEAKER,
      AMP,
      STAGE,
      FLING
    }
  }

  public final class VisionConstants {
    // april tag camera
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

    // note camera
    public static final double NOTE_CAMERA_HEIGHT_METERS = Units.inchesToMeters(13.155);
    public static final double NOTE_CAMERA_X_DISTANCE_FROM_CENTER_METERS =
        Units.inchesToMeters(14.886);
    public static final double NOTE_CAMERA_Y_DISTANCE_FROM_CENTER_METERS = Units.inchesToMeters(0);
    public static final double NOTE_CAMERA_PITCH = Units.degreesToRadians(35);
  }

  public final class DrivetrainConstants {
    // Fill in
  }

  public final class ArmConstants {
    // Fill in
    public static final double HOME_SETPOINT = homeSetpoint.get();
    public static final double CLIMB_SETPOINT = .198;
    public static final double INTAKE_SETPOINT = homeSetpoint.get();
    public static double AMP_SETPOINT = -0.27;
    public static double TRAP_SETPOINT = -0.32;
    public static final int ARM = 45;
    public static final int ARM_CANCODER_ID = 44;
    public static final double ARM_GEAR_RATIO = 266.67;
    public static final double MAG_OFFSET = -.156;
    public static final double ARM_MAX = -0.35; // -.3
  }

  public final class ShooterConstants {
    // Fill in
    public static final int LEADER = 55;
    public static final int FOLLOWER = 56;
    public static final double SETPOINT = 3000.0;
  }

  public final class ConveyorConstants {
    public static final int FEEDER_ID = 57;
    public static final int DIVRETER_ID = 58;
    public static final int BEAMBREAK_ID = 0;
    // public static final double SHOOTER_PROX_THRESHOLD = 0.55; // value was tested for, .55
    public static final double SHOOTER_PROX_THRESHOLD =
        RobotBase.isReal() ? 0.80 : -.80; // value was tested for, .80
    public static final double INTAKE_PROX_THRESHOLD =
        RobotBase.isReal() ? 0.80 : -.80; // value was tested for. .80

    public static final double DIVERTER_GEAR_RATIO = 1;
    public static final double FEEDER_GEAR_RATIO = 1;
    public static final InvertedValue diverterDirection = InvertedValue.CounterClockwise_Positive;
    public static final InvertedValue feederDirection = InvertedValue.Clockwise_Positive;
  }

  public final class IntakeConstants {
    // Fill in
    public static final int INTAKE_ID = 14;
    public static final int INTAKE_FOLLOWER_ID = 15;
    public static final int INTAKE_BEAM_BREAK_CHANNEL = 1;
  }

  public final class ClimbConstants {
    // max rotations = (distance/2pi*wheelRadius) * gearRatio
    public static final double MAX_ROTATIONS = -150;
    public static final int TOLERANCE = 1;
  }

  public final class LEDConstants {
    public static final Colors DETECT_NOTE_COLOR = Colors.White;
    public static final Colors HAS_NOTE_COLOR = Colors.White;
    public static final Colors PRIMED_SUPERSTRUCTURE = Colors.Green;

    public enum Colors {
      Red(255, 0, 0),
      Orange(255, 165, 0),
      Yellow(255, 255, 0),
      Green(0, 255, 0),
      Blue(0, 0, 255),
      Indigo(75, 0, 130),
      Violet(238, 138, 238),
      White(255, 255, 255);

      public final int r;
      public final int g;
      public final int b;

      private Colors(int r, int g, int b) {
        this.r = r;
        this.g = g;
        this.b = b;
      }
    }
  }

  public final class FlingConstants {
    public static final double BLUE_ALLIANCE_X = 1.84;
    public static final double RED_ALLIANCE_Y = 7.15;
    public static final double RED_ALLIANCE_X = 14.7;
    public static final double BLUE_ALLIANCE_Y = RED_ALLIANCE_Y;
    public static final Pose2d RED_FLING_POSE =
        new Pose2d(RED_ALLIANCE_X, RED_ALLIANCE_Y, new Rotation2d(0));
    public static final Pose2d BLUE_FLING_POSE =
        new Pose2d(BLUE_ALLIANCE_X, BLUE_ALLIANCE_Y, new Rotation2d(0));

    public static final double[] FLING_DISTANCE = {7.3, 7.726, 8.964, 9.783, 10.2, 10.7};
    public static final double[] FLING_ARM_SETPOINT = {.14, .12, .12, .12, .11, .11};
    public static final double[] FLING_SHOOTER_SETPOINT = {1500, 1500, 1750, 1750, 2000, 2000};
  }

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY,

    KANG
  }
}
