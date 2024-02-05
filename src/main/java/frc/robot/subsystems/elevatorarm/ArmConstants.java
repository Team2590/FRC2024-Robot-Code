package frc.robot.subsystems.elevatorarm;

import com.ctre.phoenix6.signals.InvertedValue;

// All Constants
public final class ArmConstants {

  public static boolean tuningMode = true;

  // Constants pertaining to the arm subsystem go here
  public static class Arm {
    public static final int ARM_ID = 4;
    public static final InvertedValue ARM_INVERT_TYPE = InvertedValue.Clockwise_Positive;

    public static final int ARM_AZIMUTH_ID = 31;
    public static final boolean ARM_AZIMUTH_INVERTED = true;
    public static final double ARM_AZIMUTH_DEGREE_OFFSET =
        313.682
            - 90.0; // Subtract 90 from the offset because the zero was obtained when the arm was
    // pointing straight up
    public static final double ARM_GEAR_RATIO = 200;
    public static final double ARM_MAX_VELOCITY = ((1.0 / 78.7) * 6380.0 * 360.0) / 60.0;

    // public static final double ARM_MAX_LIMITED_ELEVATOR_ROM = 117.24609375;
    public static final double ARM_MAX_LIMITED_ELEVATOR_ROM = 126;
    public static final double ARM_NEUTRAL_ANGLE = 110.0;
    public static final double ARM_MIN = 8.61328125;
    public static final double ARM_MAX = 236.77734375;
    public static final double ARM_SETPOINT_TOLERANCE = 3.0;
    public static final double ARM_NULL_RANGE = 310.0; // To 360 degrees

    public static final double ARM_IDLE_POSE = 90.0;
    public static final double ARM_PRE_SCORE_CUBE = 169.0;
    public static final double ARM_PRE_SCORE_CONE = 118.076;
    public static final double ARM_SLAM_CONE = 187.3828;
    public static final double ARM_PRE_SCORE_LOW = 230.0;
    public static final double ARM_FLOOR_INTAKE_CUBE = 15.0;
    public static final double ARM_DOUBLE_SUBSTATION_CONE = 178.4;
    // public static final double ARM_DOUBLE_SUBSTATION_CUBE = 211.6;
    // public static final double ARM_SINGLE_SUBSTATION_CONE = 126;
    public static final double ARM_SINGLE_SUBSTATION_CONE = 124;
    public static final double ARM_UNJAM_POSITION = 26.0;
    public static final double ARM_PRE_THROW = 25.0;
    public static final double ARM_POST_THROW = 160.0;

    public static final double ARM_KP = 0.03;
    public static final double ARM_KI = 0.0;
    public static final double ARM_KD = 0.0003;
    public static final double ARM_EFFECTIVE_MAX_VELOCITY = 405.000000;
    public static final double ARM_KG = 0.026;
    public static final double ARM_KA = 0.0;

    public static final double ARM_MAX_MOTION_ACCELERATION = 1400.0;
    public static final double ARM_MAX_MOTION_CRUISE_VELOCITY = 375.0;
  }
}
