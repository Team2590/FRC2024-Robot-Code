// spotless:off
package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.Superstructure;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import java.util.function.DoubleSupplier;

/**
 * @author Elan Ronen
 * // TODO: with angular velocity to components, check which way is front
 */
public interface ShootMath {

    class FAKE_SHOOTER {
        static double pitch = 3;
    }

    /** Acceleration due to gravity (m/s^2) */
    final double GRAVITY = 9.80;
    /** Shooter-induced projectile velocity (m/s) */
    final double SHOOT_VELOCITY = 10; // TODO: measure and set
    /** Distance from drivetrain center to end of shooter. (m) */
    final double SHOOTER_RADIUS = Units.inchesToMeters(12); // TODO: measure and set

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

        // TODO: DriverStation.getAlliance().get()
        final double SIGNED_EXTRUDE = Math.copySign(EXTRUDE, Alliance.Red == Alliance.Red ? -1 : 1);

        // TODO: DriverStation.getAlliance().get()
        /** The center of the speaker opening. */
        final Vector CENTER = new Vector(
            (Alliance.Red == Alliance.Red ? RED_X : BLUE_X) + SIGNED_EXTRUDE / 2,
            CENTER_Y,
            (LOW + HIGH)/2
        );

        // TODO: DriverStation.getAlliance().get()
        /** Vertex of the speaker opening in the +y +z direction. */
        final Vector MAX_Y_MAX_Z_VERTEX = new Vector(
            (Alliance.Red == Alliance.Red ? RED_X : BLUE_X) + SIGNED_EXTRUDE,
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

    public static Command shoot(
        Drive drive, Superstructure superstructure,
        DoubleSupplier xSupplier, DoubleSupplier ySupplier,
        Target target
    ) {
        return new SequentialCommandGroup(
            new ParallelDeadlineGroup(
                checkForHits(drive, target.surfaces),
                Commands.runOnce(superstructure::primeShooter, superstructure.getShooter()),
                snapToTarget(drive, xSupplier, ySupplier, target.point)
            ),
            Commands.runOnce(superstructure::shoot, superstructure.getShooter()),
            Commands.runOnce(() -> System.out.println("PEW PEW PEW PEW"))
        );
    }

    public static Command checkForHits(Drive drive, Triangle... triangles) {
        return Commands.waitUntil(() -> {
            final var robotPose = RobotContainer.poseEstimator.getLatestPose();
            final var robotHeading = robotPose.getRotation().getRadians();
            final var projectileInitialHeight = 0; // TODO: calculate
            final var robotVector = new Vector(robotPose.getX(), robotPose.getY(), projectileInitialHeight);
            for (final var triangle : triangles) {
                if (willHit(
                    calcRobotVelocity(drive, robotPose.getRotation().getRadians()),
                    GRAVITY,
                    SHOOT_VELOCITY,
                    robotHeading,
                    FAKE_SHOOTER.pitch, // TODO: calculate,
                    triangle.minus(robotVector)
                )) return true;
            }
            return false;
        });
    }

    public static Command snapToTarget(
        Drive drive,
        DoubleSupplier xSupplier, DoubleSupplier ySupplier,
        Vector target
    ) {
        return DriveCommands.oneJoystickDrive(drive, xSupplier, ySupplier, () -> {
            final var robotPose = RobotContainer.poseEstimator.getLatestPose();
            final var projectileInitialHeight = 0; // TODO: calculate
            final var targetShooterState = calcConstantVelocity(
                SHOOT_VELOCITY,
                target.minus(new Vector(robotPose.getX(), robotPose.getY(), projectileInitialHeight)),
                calcRobotVelocity(drive, robotPose.getRotation().getRadians()),
                GRAVITY
            );

            FAKE_SHOOTER.pitch = targetShooterState.pitch;

            return drive.snapController.calculate(
                drive.getRotation().getRadians(),
                targetShooterState.yaw
            ) * Drive.MAX_ANGULAR_SPEED;
        });
    }

    public static Vector calcRobotVelocity(Drive drive, double robotHeading) {
        final var robotAngularVelocity = SHOOTER_RADIUS * drive.currentChassisSpeeds.omegaRadiansPerSecond;
        return new Vector(
            drive.currentChassisSpeeds.vxMetersPerSecond + robotAngularVelocity * Math.cos(robotHeading + Math.PI/2),
            drive.currentChassisSpeeds.vyMetersPerSecond + robotAngularVelocity * Math.sin(robotHeading + Math.PI/2),
            0 // TODO: calculate
        );
    }

    /**
     * Calculates the yaw and pitch of the shooter from a set shooting velocity.
     * https://www.desmos.com/3d/b8ff2f55bd
     *
     * @param pv - shooting velocity
     * @param dx - x distance to target
     * @param dy - y distance to target
     * @param dz - z distance to target
     * @param rvx - initial x velocity relative to target
     * @param rvy - initial y velocity relative to target
     * @param rvz - initial z velocity relative to target
     * @param g - constant of gravity
     * @return The shooting velocity, yaw, and pitch.
     */
    public static ShootState calcConstantVelocity(double sv, Vector d, Vector v, double g) {
        final var tf = approxQuartic(
            g * g / 4,
            -v.z * g,
            d.z * g - sv * sv + v.dot(v),
            -2 * d.dot(v),
            d.dot(d),
            d.magnitude() / sv,
            10
        );

        final var projectileVelocity = d.scale(1/tf).minus(v).plus(new Vector(0, 0, g * tf / 2));

        final var yaw = Math.atan2(projectileVelocity.y, projectileVelocity.x);
        final var pitch = Math.atan2(projectileVelocity.z, Math.hypot(projectileVelocity.x, projectileVelocity.y));

        return new ShootState(sv, yaw, pitch);
    }

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
            0.01
        );
    }

    // Lightweight 3D math library

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
