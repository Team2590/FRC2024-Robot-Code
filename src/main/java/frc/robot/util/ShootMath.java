package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import java.util.function.DoubleSupplier;

/**
 * @author Elan Ronen TODO: set arm speed as z velocity
 */
public interface ShootMath {

  double DEADBAND = 0.1;

  double GRAVITY = 9.8;
  /** TODO: measure and set */
  double SHOOT_VELOCITY = 20;
  /** TODO: measure and set */
  double PROJECTILE_INITIAL_HEIGHT = 0;

  public static Command shoot(
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
        0,
        GRAVITY
      ).yaw;

      return drive.snapController.calculate(drive.getRotation().getRadians(), theta)
        * Drive.MAX_ANGULAR_SPEED;
    });
  }

  /**
   * Represents the state of the shooter. The yaw is zero when facing positive x and increases
   * counter-clockwise. The pitch is zero when parallel to the xy-plane and increases pointing up.
   */
  public static record ShootState(double velocity, double yaw, double pitch) {}

  public static record Vector(double x, double y, double z) {

    public double magnitude() {
      return Math.sqrt(this.x * this.x + this.y * this.y + this.z * this.z);
    }
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
    final var tf =
        approxQuartic(
            Math.pow(g, 2) / 4,
            -rvz * g,
            dz * g - Math.pow(pv, 2) + Math.pow(rvx, 2) + Math.pow(rvy, 2) + Math.pow(rvz, 2),
            -2 * (dx * rvx + dy * rvy + dz * rvz),
            Math.pow(dx, 2) + Math.pow(dy, 2) + Math.pow(dz, 2),
            Math.sqrt(Math.pow(dx, 2) + Math.pow(dy, 2) + Math.pow(dz, 2)) / pv,
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
      final var x4 = Math.pow(x, 4);
      final var x3 = Math.pow(x, 3);
      final var x2 = Math.pow(x, 2);
      x -= (a * x4 + b * x3 + c * x2 + d * x + e) / (4 * a * x3 + 3 * b * x2 + 2 * c * x + d);
    }
    return x;
  }

  /**
   * Calculates the velocity, yaw, and pitch of the shooter from the entry angle to the target (zero
   * is parallel to the ground).
   *
   * @param phi - entry angle
   * @param dx - x distance to target
   * @param dy - y distance to target
   * @param dz - z distance to target
   * @param rvx - initial x velocity relative to target
   * @param rvy - initial y velocity relative to target
   * @param rvz - initial z velocity relative to target
   * @param g - constant of gravity
   * @return The shooting velocity, yaw, and pitch.
   */
  public static ShootState calcEntryAngle(
      double phi, double dx, double dy, double dz, double rvx, double rvy, double rvz, double g) {
    final var dm = Math.tan(phi) * (Math.abs(dx) - Math.abs(dy) > 0 ? dx : dy);
    final var tf = Math.sqrt(-2 * (dm - dz) / g);

    final var vx = dx / tf - rvx;
    final var vy = dy / tf - rvy;
    final var vxy = Math.hypot(vx, vy);
    final var vz = 2 * (dz - dm) / tf;

    final var pv_theta = Math.atan2(vy, vx);
    final var pv_phi = Math.atan2(vz, vxy);
    final var pv = Math.hypot(vxy, vz);

    return new ShootState(pv, pv_theta, pv_phi);
  }

  public static ShootState calc2DConstantVelocity() {
    return null;
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
   * @param T0x - triangle's first point's x coordinate
   * @param T0y - triangle's first point's y coordinate
   * @param T0z - triangle's first point's z coordinate
   * @param T1x - triangle's second point's x coordinate
   * @param T1y - triangle's second point's y coordinate
   * @param T1z - triangle's second point's z coordinate
   * @param T2x - triangle's third point's x coordinate
   * @param T2y - triangle's third point's y coordinate
   * @param T2z - triangle's third point's z coordinate
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
      double T0x,
      double T0y,
      double T0z,
      double T1x,
      double T1y,
      double T1z,
      double T2x,
      double T2y,
      double T2z) {
    final var N = cross(T1x - T0x, T1y - T0y, T1z - T0z, T2x - T1x, T2y - T1y, T2z - T1z);
    final var A = N.z * g;
    final var X = rvx * pv * Math.cos(pv_phi) * Math.cos(pv_theta);
    final var Y = rvy * pv * Math.cos(pv_phi) * Math.sin(pv_theta);
    final var Z = rvz * pv * Math.sin(pv_phi);
    final var B = dot(N.x, N.y, N.z, X, Y, Z);
    final var tf = (B + Math.sqrt(B * B - 2 * A * dot(N.x, N.y, N.z, T0x, T0y, T0z))) / A;
    final var P = new Vector(X * tf, Y * tf, Z * tf - g * tf * tf / 2);
    final var triangle0 =
        cross(T1x - T0x, T1y - T0y, T1z - T0z, T2x - T0x, T2y - T0y, T2z - T0z).magnitude();
    final var triangle1 =
        cross(P.x - T0x, P.y - T0y, P.z - T0z, P.x - T1x, P.y - T1y, P.z - T1z).magnitude();
    final var triangle2 =
        cross(P.x - T1x, P.y - T1y, P.z - T1z, P.x - T2x, P.y - T2y, P.z - T2z).magnitude();
    final var triangle3 =
        cross(P.x - T2x, P.y - T2y, P.z - T2z, P.x - T0x, P.y - T0y, P.z - T0z).magnitude();
    return MathUtil.isNear(triangle0, triangle1 + triangle2 + triangle3, 0.0001);
  }

  /**
   * The vector dot product.
   *
   * @param ax - first point's x coordinate
   * @param ay - first point's y coordinate
   * @param az - first point's z coordinate
   * @param bx - second point's x coordinate
   * @param by - second point's y coordinate
   * @param bz - second point's z coordinate
   * @return The dot product of the two vectors.
   */
  public static double dot(double ax, double ay, double az, double bx, double by, double bz) {
    return ax * bx + ay * by + az * bz;
  }
  ;

  /**
   * The vector cross product.
   *
   * @param ax - first point's x coordinate
   * @param ay - first point's y coordinate
   * @param az - first point's z coordinate
   * @param bx - second point's x coordinate
   * @param by - second point's y coordinate
   * @param bz - second point's z coordinate
   * @return The cross product of the two vectors.
   */
  public static Vector cross(double ax, double ay, double az, double bx, double by, double bz) {
    return new Vector(ay * bz - az * by, az * bx - ax * bz, ax * by - ay * bx);
  }

  public static void main(String[] args) {
    System.out.println(calcConstantVelocity(12.4, 9, 8, 4, 5, -5, 3.5, 9.8));
  }

}
