// spotless:off
package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.Superstructure;
import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.FieldConstants.Targets;
import frc.robot.subsystems.drive.Drive;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

/**
 * An API for calculation of shot trajectory, including while moving.
 * 
 * <p>HOW TO:
 *  <p>
 *    Update shoot velocity:
 * 
 *    Do a stationary shot. If it shoots too high, decrease constant, if it
 *    shoots too low, increase constant. Remember that the target is the
 *    center of the speaker *opening*, not the center of the inside of the speaker.
 *  </p>
 *  <p>
 *    Fix strafing/stooping misses:
 * 
 *    Use mulitpliers to make the formula think the robot is going faster/slower
 *    so that it over/undercompensates. If the multiplier varies based on robot
 *    speed, create an interpolation table that has a direct relationship from
 *    robot velocity to multipier.
 *  </p>
 * 
 * @author Elan Ronen
 * @author Ian Keller
 * 
 */
public interface ShootMath {
    // TODO: instead of flipping random signs in the equation, flip the signs of the chassis velocity

    // superstructure API █████████████████████████████████████████████████████████████████████████

    /** Gets the current pitch of the shooter */
    public static double getShooterPitch(Superstructure superstructure) {
        return encoderTicksToRadians(superstructure.getArm().getAbsolutePosition());
    }

    /** Set the current pitch of the shooter */
    public static void setShooterPitch(Superstructure superstructure, double pitch) {
        superstructure.getArm().setPosition(radiansToEncoderTicks(pitch));
    }

    /** Convert arm encoder ticks to radians, factoring in gearing */
    public static double encoderTicksToRadians(double encoderTicks) {
        return Units.degreesToRadians(encoderTicks / 0.155 * 54);
    }

    /** Convert radians to arm encoder ticks, factoring in gearing */
    public static double radiansToEncoderTicks(double radians) {
        return Units.radiansToDegrees(radians) / 54 * 0.155;
    }

    /** Get height of the release point of the shooter */
    public static double getProjectileHeight() {
        return Units.inchesToMeters(25.5); // TODO: calculate
    }

    public static double getProjectileZVelocity() {
        return 0; // TODO: calculate
    }

    // field, robot, and Earth constants ██████████████████████████████████████████████████████████

    LoggedTunableNumber loggedShootVelocity = new LoggedTunableNumber("ShootMath/shootVelocity", 12.35);

    /** Acceleration due to gravity (m/s^2) */
    final double GRAVITY = 9.80;
    /** Shooter-induced projectile velocity (m/s) */
    //final double SHOOT_VELOCITY = 15; // TODO: measure and set
    /** Distance from drivetrain center to end of shooter. (m) */
    final double SHOOTER_RADIUS = Units.inchesToMeters(0); // TODO: measure and set (...or not)

    boolean redAlliance = DriverStation.getAlliance().get() == Alliance.Red;

    /** Speaker coords. (m) */
    public interface Speaker {

        /** Lowest edge of opening. (m) */
        final double LOW = Units.inchesToMeters(78);
        /** Highest edge of opening. (m) */
        final double HIGH = Units.inchesToMeters(83);
        /** Length into the field. (m) */
        final double EXTRUDE = Units.inchesToMeters(18);
        /** Width across the field. (m) */
        final double WIDTH = Units.inchesToMeters(41.625);
        /** Center of speakers' y coordinates. (m) */
        final double CENTER_Y = Units.inchesToMeters(218.42);
        /** Red speaker's x coordinate. (m) */
        final double RED_X = Units.inchesToMeters(652.73);
        /** Blue speaker's x coordinate. (m) */
        final double BLUE_X = Units.inchesToMeters(-1.5);

        final double SIGNED_EXTRUDE = Math.copySign(EXTRUDE, redAlliance ? -1 : 1);

        /** The center of the speaker opening. */
        final Vector CENTER = new Vector(
            (redAlliance ? RED_X : BLUE_X) + SIGNED_EXTRUDE / 2,
            CENTER_Y,
            (LOW + HIGH)/2
        );

        /** Vertex of the speaker opening in the +y +z direction. */
        final Vector MAX_Y_MAX_Z_VERTEX = new Vector(
            (redAlliance ? RED_X : BLUE_X) + SIGNED_EXTRUDE,
            CENTER_Y + WIDTH / 2,
            HIGH
        );
        /** Vertex of the speaker opening in the +y -z direction. */
        final Vector MAX_Y_MIN_Z_VERTEX = MAX_Y_MAX_Z_VERTEX.minus(new Vector(SIGNED_EXTRUDE, 0, HIGH - LOW));
        /** Vertex of the speaker opening in the -y -z direction. */
        final Vector MIN_Y_MIN_Z_VERTEX = MAX_Y_MAX_Z_VERTEX.minus(new Vector(SIGNED_EXTRUDE, WIDTH, HIGH - LOW));
        /** Vertex of the speaker opening in the -y +z direction. */
        final Vector MIN_Y_MAX_Z_VERTEX = MAX_Y_MAX_Z_VERTEX.minus(new Vector(0, WIDTH, 0));

        /** The triangle of the speaker opening in the +y +z direction. */
        final Triangle MAX_YZ_Triangle = new Triangle(MAX_Y_MAX_Z_VERTEX, MAX_Y_MIN_Z_VERTEX, MIN_Y_MAX_Z_VERTEX);

        /** The triangle of the speaker opening in the -y -z direction. */
        final Triangle MIN_YZ_Triangle = new Triangle(MIN_Y_MIN_Z_VERTEX, MIN_Y_MAX_Z_VERTEX, MAX_Y_MIN_Z_VERTEX);

        final Target target = new Target(CENTER, MAX_YZ_Triangle, MIN_YZ_Triangle);
    }

    // driving commands ███████████████████████████████████████████████████████████████████████████

    double DEADBAND = 0.1;

    /**
     * Maps an input from -1 to 1 to an output from -1 to 1.
     *
     * @param x [-1, 1]
     * @return [-1, 1]
     */
    private static double scale(double x) {
        return Math.copySign(x * x, x);
    }

    /**
     * Field relative drive command using one joystick (controlling linear).
     * <p>Desmos: https://www.desmos.com/calculator/cswpncuxr2
     *
     * @param drive - the drive subsystem
     * @param xSupplier - function to supply x values [-1, 1]
     * @param ySupplier - function to supply y values [-1, 1]
     * @param angularVelocitySupplier - function to supply the angular velocity for the chassis speeds
     * @return the drive command
     */
    public static Command oneJoystickDrive(
        Drive drive,
        DoubleSupplier xSupplier, DoubleSupplier ySupplier,
        DoubleSupplier angularVelocitySupplier
    ) {
        return Commands.run(() -> {
            final var x = xSupplier.getAsDouble();
            final var y = ySupplier.getAsDouble();

            final var magnitude = Math.hypot(x, y);

            if (magnitude < DEADBAND) {
                drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(
                    0, 0,
                    angularVelocitySupplier.getAsDouble(), drive.getRotation()
                ));
                return;
            }

            final var newMagnitude = scale(
                (magnitude - Math.copySign(DEADBAND, magnitude)) / (1 - DEADBAND)
            );

            final var newX = newMagnitude > 1
                ? Math.signum(x) / Math.hypot(y / x, 1)
                : newMagnitude * x / magnitude;

            final var newY = newMagnitude > 1
                ? Math.signum(y) / Math.hypot(x / y, 1)
                : newMagnitude * y / magnitude;

            drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(
                newX * drive.getMaxLinearSpeedMetersPerSec(),
                newY * drive.getMaxLinearSpeedMetersPerSec(),
                angularVelocitySupplier.getAsDouble(),
                drive.getRotation()
            ));
        }, drive);
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
        DoubleSupplier xSupplier, DoubleSupplier ySupplier,
        DoubleSupplier omegaSupplier
    ) {
        return oneJoystickDrive(
            drive,
            xSupplier, ySupplier,
            () -> scale(MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND))
                * drive.getMaxAngularSpeedRadPerSec()
        );
    }

    // shoot commands █████████████████████████████████████████████████████████████████████████████

    /**
     * Shoot into the speaker
     * 
     * @param drive - drive instance
     * @param superstructure - superstructure instance
     * @param xSupplier - joystick x value
     * @param ySupplier - joystick y value
     * @param target - coordinate target from <code>ShootMath</code> class
     * @param fire - condition to fire
     * @return the command
     */
    public static Command shoot(
        Drive drive, Superstructure superstructure,
        DoubleSupplier xSupplier, DoubleSupplier ySupplier,
        Target target,
        Command fire
    ) {
        return new SequentialCommandGroup(
            new ParallelDeadlineGroup(
                fire,
                snapToTarget(drive, superstructure, xSupplier, ySupplier, target.point),
                Commands.runOnce(superstructure::prep, superstructure),
                Commands.runOnce(() -> superstructure.led.setColor(LEDConstants.Colors.Violet)),
                new SequentialCommandGroup(
                    checkForHits(drive, superstructure, target.surfaces),
                    Commands.runOnce(() -> superstructure.led.setColor(LEDConstants.Colors.Yellow))
                )
            ),
            Commands.runOnce(superstructure::shootBlind, superstructure),
            Commands.runOnce(() -> {
                System.out.println("DRIVE X: " + drive.getCurrentChassisSpeeds().vxMetersPerSecond);
                System.out.println("DRIVE Y: " + drive.getCurrentChassisSpeeds().vyMetersPerSecond);
            }),
            Commands.runOnce(() -> System.out.println("PEW PEW PEW PEW")),
            Commands.waitUntil(() -> !superstructure.getConveyor().hasNote())
        );
    }

    /** Get robot heading */
    public static double getHeading(double heading) {
        return radianBand(redAlliance ? heading : heading + Math.PI);
    }

    public static Command checkForHits(Drive drive, Superstructure superstructure, Triangle... triangles) {
        return Commands.waitUntil(() -> {
            final var robotPose = RobotContainer.poseEstimator.getLatestPose();
            final var robotHeading = getHeading(robotPose.getRotation().getRadians());
            final var robotVector = new Vector(robotPose.getX(), robotPose.getY(), getProjectileHeight());
            for (final var triangle : triangles) {
                if (willHit(
                    calcRobotVelocity(drive, robotHeading),
                    GRAVITY,
                    loggedShootVelocity.get(),
                    robotHeading,
                    getShooterPitch(superstructure),
                    triangle.minus(robotVector)
                )) return true;
            }
            return false;
        });
    }

    LoggedTunableNumber maxRange = new LoggedTunableNumber("ShootMath/maxRange",3);

    /**
     * Checks if the shooter is ready to fire.
     * <p>The shooter is ready iff the target is in range, the snap controller is at setpoint, and the arm is at setpoint.
     * 
     * @param drive - drive instance
     * @param superstructure - superstructure instance
     * @return shooter status
     */
    public static boolean ready(Drive drive, Superstructure superstructure) {
        return RobotContainer.poseEstimator.distanceToTarget(Targets.SPEAKER) <= maxRange.get() 
            && drive.snapControllerAtSetpoint() 
            && superstructure.getArm().atSetpoint();
    }

    /**
     * Snap to the target, adjusted for velocity
     * 
     * @param drive - drive instance
     * @param superstructure - superstructure instance
     * @param xSupplier - joystick x value
     * @param ySupplier - joystick y value
     * @param target - target to snap to
     * @return the command
     */
    public static Command snapToTarget(
        Drive drive, Superstructure superstructure,
        DoubleSupplier xSupplier, DoubleSupplier ySupplier,
        Vector target
    ) {
        return oneJoystickDrive(drive, xSupplier, ySupplier, () -> {
            final var robotPose = RobotContainer.poseEstimator.getLatestPose();
            final var robotHeading = getHeading(robotPose.getRotation().getRadians());
            final var targetShooterState = calcConstantVelocity(
                loggedShootVelocity.get(),
                target.minus(new Vector(robotPose.getX(), robotPose.getY(), getProjectileHeight())),
                calcRobotVelocityFAKE(drive, robotHeading),
                GRAVITY
            );

            Logger.recordOutput("ShootMath/shooterTargetPitch", targetShooterState.pitch);
            setShooterPitch(superstructure, targetShooterState.pitch);

            Logger.recordOutput("ShootMath/robotHeading", robotHeading);
            Logger.recordOutput("ShootMath/targetHeading", radianBand(targetShooterState.yaw));
            Logger.recordOutput("ShootMath/driveVelocityX", drive.getCurrentChassisSpeeds().vxMetersPerSecond);
            Logger.recordOutput("ShootMath/driveVelocityY", drive.getCurrentChassisSpeeds().vyMetersPerSecond);

            return drive.snapController.calculate(robotHeading, radianBand(targetShooterState.yaw))
                * drive.getMaxAngularSpeedRadPerSec();
        });
    }

    LoggedTunableNumber loggedStrafeMultiplier = new LoggedTunableNumber("ShootMath/strafeMultiplier", 3);
    LoggedTunableNumber loggedStoopMultiplier = new LoggedTunableNumber("ShootMath/stoopMultiplier", 2.5);

    // shoot commands █████████████████████████████████████████████████████████████████████████████

    // multiplier interpolation
    double[] stoopKeys = {0, -1.4, -2, -3, -3.75};
    double[] stoopValues = {1, 2, 2.3, 2.5, 3};
    LookupTable stoopInterpolation = new LookupTable(stoopKeys, stoopValues);
    double[] strafeKeys = {};
    double[] strafeValues = {};
    LookupTable strafeInterpolation = new LookupTable(stoopKeys, stoopValues);

    public static Vector calcRobotVelocityFAKE(Drive drive, double robotHeading) {
        final var chassisSpeeds = drive.getCurrentChassisSpeeds();
        final var robotAngularVelocity = SHOOTER_RADIUS * chassisSpeeds.omegaRadiansPerSecond;
        return new Vector(
            // chassisSpeeds.vxMetersPerSecond * loggedStoopMultiplier.get() + robotAngularVelocity * Math.cos(robotHeading + Math.PI/2),
            chassisSpeeds.vxMetersPerSecond * stoopInterpolation.getValue(chassisSpeeds.vxMetersPerSecond) + robotAngularVelocity * Math.cos(robotHeading + Math.PI/2),
            chassisSpeeds.vyMetersPerSecond * loggedStrafeMultiplier.get() + robotAngularVelocity * Math.sin(robotHeading + Math.PI/2),
            getProjectileZVelocity()
        );
    }

    public static Vector calcRobotVelocity(Drive drive, double robotHeading) {
        final var chassisSpeeds = drive.getCurrentChassisSpeeds();
        final var robotAngularVelocity = SHOOTER_RADIUS * chassisSpeeds.omegaRadiansPerSecond;
        return new Vector(
            chassisSpeeds.vxMetersPerSecond + robotAngularVelocity * Math.cos(robotHeading + Math.PI/2),
            chassisSpeeds.vyMetersPerSecond + robotAngularVelocity * Math.sin(robotHeading + Math.PI/2),
            getProjectileZVelocity()
        );
    }

    /**
     * Calculates the yaw and pitch of the shooter from a set shooting velocity.
     * <p>Desmos: https://www.desmos.com/3d/b8ff2f55bd
     * 
     * @param sv - shooter velocity
     * @param d - distances to target
     * @param v - velocities relative to target
     * @param g - constant of gravity
     * @return The shooter velocity, yaw, and pitch.
     */
    public static ShootState calcConstantVelocity(double sv, Vector d, Vector v, double g) {
        final var tf = approxQuartic(
            g * g / 4,
            -v.z * g,
            d.z * g - sv * sv + v.dot(v),
            (redAlliance ? 2 : -2) * d.dot(v),
            d.dot(d),
            d.magnitude() / sv,
            10
        );

        final var projectileVelocity = (redAlliance ? d.scale(1/tf).plus(v) : d.scale(1/tf).minus(v))
            .plus(new Vector(0, 0, g * tf / 2));

        final var yaw = Math.atan2(projectileVelocity.y, projectileVelocity.x);
        final var pitch = Math.atan2(projectileVelocity.z, Math.hypot(projectileVelocity.x, projectileVelocity.y));

        return new ShootState(sv, yaw, pitch);
    }

/**
 *  ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢿⣶⣶⣤⣄⣀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
 * ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣀⣠⣤⣽⡿⠿⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
 * ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣤⣤⠶⠟⠛⠉⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
 * ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠈⠛⠷⠶⢤⣤⣀⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
 * ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠰⣶⣶⣶⡶⠀⠀⠀⠉⠻⣿⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
 * ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢰⣾⣿⣟⠋⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
 * ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠈⠉⠉⠙⠻⡇⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
 * ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⣀⡀⣀⣀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
 * ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣀⣤⣤⡶⠶⠶⠟⠿⠿⠛⠋⠋⠛⠛⠛⠳⠶⣤⣤⣤⣤⣤⣄⣤⣶⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
 * ⢀⠀⠀⠀⠀⠀⠀⠀⠀⣀⣠⣶⣴⠶⠿⠛⠋⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠈⠃⠀⠉⠉⠋⣰⡿⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
 * ⢸⣿⠾⠿⠿⠟⠛⠛⠛⠉⠉⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⣾⠟⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
 * ⠈⣿⣄⠀⠀⠀⠀⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⣾⠏⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
 * ⠀⠈⠻⣦⣀⠀⣼⡟⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠈⢿⡋⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
 * ⠀⠀⠀⠈⠙⢻⣿⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢸⣇⣀⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
 * ⠀⠀⠀⠀⠀⠈⣿⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⣀⣤⠀⣀⣾⡯⠽⡛⠛⠳⣦⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
 * ⠀⠀⠀⠀⠀⠀⣿⡆⠀⠀⠀⠀⠀⠀⠀⣀⣀⡀⠀⠀⠀⠀⠀⠀⢿⣶⠴⠶⠛⠛⠛⠉⣼⣾⣿⣿⡋⠉⠉⠻⠂⠈⢿⣦⣀⣠⣤⣤⣤⣄⣀⠀⠀⠀⠀
 * ⠀⠀⠀⢀⣤⣶⣿⣥⣤⡛⠛⠿⠿⠿⠿⠿⠛⠁⠀⠀⣀⣤⡀⠀⠀⠀⠀⠀⠀⢀⣠⣴⡿⠷⠿⣷⣍⠉⠓⠒⠀⠀⠀⠈⢻⡉⠉⠉⠙⡋⠉⠙⠳⣦⠀
 * ⠀⠀⠰⢋⡭⠖⣹⣿⡿⣿⣤⣄⣀⣀⣀⠀⠀⠀⠀⠀⠙⠋⠁⠀⠀⠀⠀⣠⣴⡿⠋⠁⠀⠀⠀⢼⠟⠀⠀⠀⠀⠀⠀⢰⣾⣿⣥⣴⣦⣶⣶⠀⢠⢿⡧
 * ⠀⢠⡖⠋⣀⡼⠛⣽⡿⠻⠉⠉⠉⠉⠙⠻⢶⣦⣄⡀⠀⠀⠀⠀⠀⢀⣴⡿⠋⠀⠀⠀⠀⠀⢀⣦⠀⠀⠀⠀⠀⠀⣴⣿⠏⠀⠀⣠⣴⡿⠏⠀⣼⣾⡇
 * ⠀⠈⢀⣴⠟⠀⢸⡟⠀⠀⠀⠀⠀⠀⠀⠀⠀⠈⠙⢿⣦⡀⠀⠀⢀⣼⡿⠁⠀⠀⠀⠀⠀⣠⣾⠃⢀⣠⣤⣴⣶⡿⠟⢁⣴⣾⠟⠋⠁⠀⢀⣼⣿⠏⠀
 * ⠀⠀⠘⠛⠀⠀⢸⣧⣀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠙⣿⣶⡶⢾⡟⠁⠀⠀⣀⣤⡶⠿⠟⠙⠛⠛⠉⠉⠉⠀⣠⣴⡿⠏⠀⠀⢀⣠⣴⣿⡿⠛⠀⠀
 * ⠀⠀⠀⠀⠀⠀⠀⠈⠙⠓⠒⠦⣤⣀⠀⠀⠀⣀⠀⠀⢀⣬⡿⠀⠘⠷⢶⡶⠾⠛⠉⠀⠀⠀⠀⠀⠀⠀⠀⠀⠸⣿⣿⣐⣦⣲⣿⣿⡿⠛⠋⠀⠀⠀⠀
 * ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠉⠛⠛⠲⠾⠿⠟⠛⠋⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀  ⠀⠀⠀⠀⠀⠀⠀⠀⠉⠙⠛⠛⠋⠉⠁⠀⠀⠀⠀⠀
 * ⠀⠀
 *  Have a break from this nightmare
 */

    /**
     * Uses Newton's Method to approximate the roots of a quartic.
     *
     * @param a - x^4
     * @param b - x^3
     * @param c - x^2
     * @param d - x^1
     * @param e - x^0
     * @param x - initial approximation
     * @param n - iterations of the algorithm
     * @return An approximate root.
     */
    public static double approxQuartic(
        double a, double b, double c, double d, double e,
        double x, double n
    ) {
        for (; n > 0; n--) {
            final var x2 = x * x;
            final var x3 = x2 * x;
            final var x4 = x3 * x;
            x -= (a * x4 + b * x3 + c * x2 + d * x + e) / (4 * a * x3 + 3 * b * x2 + 2 * c * x + d);
        }
        return x;
    }

    /**
     * Checks if a projectile will hit a triangle.
     *
     * @param rvx - initial x velocity relative to target
     * @param rvy - initial y velocity relative to target
     * @param rvz - initial z velocity relative to target
     * @param g - constant of gravity
     * @param pv - shooting velocity
     * @param pv_theta - shooter yaw
     * @param pv_phi - shooter pitch
     * @param triangle - distance to triangle to check collision with
     * @return Whether the projectile will hit the triangle.
     */
    public static boolean willHit(
        Vector v,
        double g,
        double pv, double pv_theta, double pv_phi,
        Triangle triangle
    ) {
        final var N = triangle.p1.minus(triangle.p0).cross(triangle.p2.minus(triangle.p1));
        final var A = N.z * g;
        final var V = v.plus(new Vector(
            pv * Math.cos(pv_phi) * Math.cos(pv_theta),
            pv * Math.cos(pv_phi) * Math.sin(pv_theta),
            pv * Math.sin(pv_phi)
        ));
        final var B = N.dot(V);
        final var tf = (B + Math.sqrt(B * B - 2 * A * N.dot(triangle.p0))) / A;
        final var P = V.scale(tf).minus(new Vector(0, 0, g * tf * tf / 2));
        final var parallelogram0 = triangle.p1.minus(triangle.p0).cross(triangle.p2.minus(triangle.p0)).magnitude();
        final var parallelogram1 = P.minus(triangle.p0).cross(P.minus(triangle.p1)).magnitude();
        final var parallelogram2 = P.minus(triangle.p1).cross(P.minus(triangle.p2)).magnitude();
        final var parallelogram3 = P.minus(triangle.p2).cross(P.minus(triangle.p0)).magnitude();

        return MathUtil.isNear(
            parallelogram0,
            parallelogram1 + parallelogram2 + parallelogram3,
            0.05
        );
    }

    // 3d math library ████████████████████████████████████████████████████████████████████████████

    double TAU = 2 * Math.PI;

    public static double radianBand(double radians) {
        return MathUtil.inputModulus(radians, 0, TAU);
    }

    /**
     * Represents the state of the shooter. The yaw is zero when facing positive x and increases
     * counter-clockwise. The pitch is zero when parallel to the xy-plane and increases pointing up.
     */
    public static record ShootState(double velocity, double yaw, double pitch) {}

    public static record Vector(double x, double y, double z) {

        public double magnitude() {
            return Math.sqrt(x * x + y * y + z * z);
        }

        public Vector scale(double scalar) {
            return new Vector(
                x * scalar,
                y * scalar,
                z * scalar
            );
        }

        public Vector minus(Vector other) {
            return new Vector(
                x - other.x,
                y - other.y,
                z - other.z
            );
        }

        public Vector plus(Vector other) {
            return new Vector(
                x + other.x,
                y + other.y,
                z + other.z
            );
        }

        public double dot(Vector other) {
            return x * other.x + y * other.y + z * other.z;
        }

        public Vector cross(Vector other) {
            return new Vector(
                y * other.z - z * other.y,
                z * other.x - x * other.z,
                x * other.y - y * other.x
            );
        }

    }

    public static record Triangle(Vector p0, Vector p1, Vector p2) {

        public Triangle minus(Vector vector) {
            return new Triangle(p0.minus(vector), p1.minus(vector), p2.minus(vector));
        }

    }

    public static record Target(Vector point, Triangle... surfaces) {}

}
// spotless:on
