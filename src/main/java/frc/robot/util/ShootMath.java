package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.Superstructure;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ShootCommand;
import frc.robot.subsystems.drive.Drive;
import java.util.function.DoubleSupplier;

/**
 * @author Elan Ronen TODO: set arm speed as z velocity
 */
public interface ShootMath {

  double DEADBAND = 0.1;

  double GRAVITY = 9.8;
  double SHOOT_VELOCITY = 20; // TODO: measure and set
  double PROJECTILE_INITIAL_HEIGHT = 0; // TODO: measure and set

  Triangle speaker0 = new Triangle(new Vector(0,0,0),new Vector(0,0,0),new Vector(0,0,0));
  Triangle speaker1 = new Triangle(new Vector(0,0,0),new Vector(0,0,0),new Vector(0,0,0));

  public static Command shoot(
    Drive drive, Superstructure superstructure,
    DoubleSupplier xSupplier, DoubleSupplier ySupplier,
    Pose3d target
  ) {
    return new SequentialCommandGroup(
      new ParallelDeadlineGroup(
        checkForHits(drive),
        snapToPosition(drive, xSupplier, ySupplier, target)
      ),
      new ShootCommand(superstructure, DEADBAND)
    );
  }

  public static Command checkForHits(Drive drive) {
    return Commands.waitUntil(() -> {
      final var robotPose = RobotContainer.poseEstimator.getLatestPose();
      return willHit(
        drive.currentChassisSpeeds.vxMetersPerSecond,
        drive.currentChassisSpeeds.vyMetersPerSecond,
        0, // TODO: calculate
        GRAVITY,
        SHOOT_VELOCITY,
        robotPose.getRotation().getRadians(),
        Math.PI/4, // TODO: calculate
        speaker0
      ) || willHit(
        drive.currentChassisSpeeds.vxMetersPerSecond,
        drive.currentChassisSpeeds.vyMetersPerSecond,
        0, // TODO: calculate
        GRAVITY,
        SHOOT_VELOCITY,
        robotPose.getRotation().getRadians(),
        Math.PI/4, // TODO: calculate
        speaker1
      );
    });
  }

  public static Command snapToPosition(
    Drive drive,
    DoubleSupplier xSupplier, DoubleSupplier ySupplier,
    Pose3d target
  ) {
    return DriveCommands.oneJoystickDrive(drive, xSupplier, ySupplier, () -> {
      final var robotPose = RobotContainer.poseEstimator.getLatestPose();
      final var theta = calcConstantVelocity(
        SHOOT_VELOCITY,
        target.getX() - robotPose.getX(),
        target.getY() - robotPose.getY(),
        target.getZ() - PROJECTILE_INITIAL_HEIGHT,
        drive.currentChassisSpeeds.vxMetersPerSecond,
        drive.currentChassisSpeeds.vyMetersPerSecond,
        0, //TODO: calculate
        GRAVITY
      ).yaw;

      return drive.snapController.calculate(drive.getRotation().getRadians(), theta)
        * Drive.MAX_ANGULAR_SPEED;
    });
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
  public static ShootState calcConstantVelocity(
      double pv, double dx, double dy, double dz, double rvx, double rvy, double rvz, double g) {
    final var sumsqrs = dx * dx + dy * dy + dz * dz;
    final var tf =
        approxQuartic(
            Math.pow(g, 2) / 4,
            -rvz * g,
            dz * g - pv * pv + rvx * rvx + rvy * rvz * rvz,
            -2 * (dx * rvx + dy * rvy + dz * rvz),
            sumsqrs,
            Math.sqrt(sumsqrs) / pv,
            10);

    final var vx = dx / tf - rvx;
    final var vy = dy / tf - rvy;
    final var vz = dz / tf + g * tf / 2 - rvz;

    final var pv_theta = Math.atan2(vy, vx);
    final var pv_phi = Math.atan2(vz, Math.hypot(vx, vy));

    return new ShootState(pv, pv_theta, pv_phi);
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
      double a, double b, double c, double d, double e, double x, double n) {
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
   * @param triangle - triangle to check collision with
   * @return Whether the projectile will hit the triangle.
   */
  public static boolean willHit(
      double rvx,
      double rvy,
      double rvz,
      double g,
      double pv,
      double pv_theta,
      double pv_phi,
      Triangle triangle) {
    final var N = cross(triangle.p1.minus(triangle.p0), triangle.p2.minus(triangle.p1));
    final var A = N.z * g;
    final var X = rvx * pv * Math.cos(pv_phi) * Math.cos(pv_theta);
    final var Y = rvy * pv * Math.cos(pv_phi) * Math.sin(pv_theta);
    final var Z = rvz * pv * Math.sin(pv_phi);
    final var B = dot(N, new Vector(X, Y, Z));
    final var tf = (B + Math.sqrt(B * B - 2 * A * dot(N, triangle.p0))) / A;
    final var P = new Vector(X * tf, Y * tf, Z * tf - g * tf * tf / 2);
    final var parallelogram0 = cross(triangle.p1.minus(triangle.p0), triangle.p2.minus(triangle.p0)).magnitude();
    final var parallelogram1 = cross(P.minus(triangle.p0), P.minus(triangle.p1)).magnitude();
    final var parallelogram2 = cross(P.minus(triangle.p1), P.minus(triangle.p2)).magnitude();
    final var parallelogram3 = cross(P.minus(triangle.p2), P.minus(triangle.p0)).magnitude();

    return MathUtil.isNear(parallelogram0, parallelogram1 + parallelogram2 + parallelogram3, 0.0001);
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

    public Vector minus(Vector other) {
      return new Vector(x - other.x, y - other.y, z - other.z);
    }

  }

  public static record Triangle(Vector p0, Vector p1, Vector p2) {}

  public static double dot(Vector a, Vector b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
  }

  public static Vector cross(Vector a, Vector b) {
    return new Vector(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
  }

  public static void main(String[] args) {
    System.out.println(calcConstantVelocity(12.4, 9, 8, 4, 5, -5, 3.5, 9.8));
  }

}
