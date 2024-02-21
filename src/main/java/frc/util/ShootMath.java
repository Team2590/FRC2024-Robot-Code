package frc.util;

/**
 * @author Elan Ronen
 * 
 * TODO: add formula for shooting with entry angle
 * TODO: finish simplifying equation for t_f
 * TODO: find inequality to check if a target is "within reach"
 */
public interface ShootMath {

    /**
     * Represents the state of the shooter.
     * The yaw is zero when facing positive x and increases counter-clockwise.
     * The pitch is zero when parallel to the xy-plane and increases pointing up.
     */
    public static record ShootState(double velocity, double yaw, double pitch) {};

    /**
     * Calculates the yaw and pitch of the shooter from a set shooting velocity.
     * https://www.desmos.com/3d/b8ff2f55bd
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
        double pv,
        double dx, double dy, double dz,
        double rvx, double rvy, double rvz,
        double g
    ) {
        final var tf = approxQuartic(
            Math.pow(g, 2) / 4,
            -rvz * g,
            dz * g - Math.pow(pv, 2) + Math.pow(rvx, 2) + Math.pow(rvy, 2) + Math.pow(rvz, 2),
            -2 * (dx * rvx + dy * rvy + dz * rvz),
            Math.pow(dx, 2) + Math.pow(dy, 2) + Math.pow(dz, 2),
            Math.hypot(Math.hypot(dx, dy), dz) / pv,
            10
        );

        final var vx = dx / tf - rvx;
        final var vy = dy / tf - rvy;
        final var vz = dz / tf + g * tf / 2 - rvz;

        final var pv_theta = Math.atan2(vy, vx);
        final var pv_phi = Math.atan2(vz, Math.hypot(vx, vy));

        return new ShootState(pv, pv_theta, pv_phi);
    }

    /**
     * Uses Newton's Method to approximate the roots of a quartic.
     * @param a - x^4
     * @param b - x^3
     * @param c - x^2
     * @param d - x^1
     * @param e - x^0
     * @param x - initial approximation
     * @param n - iterations of the algorithm
     * @return An approximate root.
     */
    public static double approxQuartic(double a, double b, double c, double d, double e, double x, double n) {
        for (; n > 0; n--) {
            final var x4 = Math.pow(x, 4);
            final var x3 = Math.pow(x, 3);
            final var x2 = Math.pow(x, 2);
            x -= (a*x4 + b*x3 + c*x2 + d*x + e) / (4*a*x3 + 3*b*x2 + 2*c*x + d);
        }
        return x;
    }

    /**
     * Solves for the roots of a general quartic. DOES NOT WORK
     * https://www.desmos.com/calculator/gamythajrx
     * @param a - x^4
     * @param b - x^3
     * @param c - x^2
     * @param d - x^1
     * @param e - x^0
     * @return The four roots.
     */
    public static double[] solveQuartic(double a, double b, double c, double d, double e) {
        final var a3 = b / a;
        final var a2 = c / a;
        final var a1 = d / a;
        final var a0 = e / a;

        final var Q3 = (3 * (a1 * a3 - 4 * a0) - Math.pow(a2, 2)) / 9;
        final var R3 = -9 * a2 * (a1 * a3 - 4 * a0) - 27 * (4 * a2 * a0 - Math.pow(a1, 2) - Math.pow(a3, 2) * a0) + 2 * Math.pow(a2, 3);
        final var y1 = 2 * Math.sqrt(-Q3) * Math.cos(Math.acos(R3 / Math.sqrt(-Math.pow(Q3, 3))) / 3) + a2 / 3;

        final var R = Math.sqrt(Math.pow(a3, 2) / 4 - a2 + y1);
        final var D = R == 0 ?
            Math.sqrt(3 * Math.pow(a3, 2) / 4 - 2 * a2 + 2 * Math.sqrt(Math.pow(y1, 2) - 4 * a0)) :
            Math.sqrt(3 * Math.pow(a3, 2) / 4 - Math.pow(R, 2) - 2 * a2 + (4 * a3 * a2 - 8 * a1 - Math.pow(a3, 3)) / (4 * R))
        ;
        final var E = R == 0 ?
            Math.sqrt(3 * Math.pow(a3, 2) / 4 - 2 * a2 - 2 * Math.sqrt(Math.pow(y1, 2) - 4 * a0)) :
            Math.sqrt(3 * Math.pow(a3, 2) / 4 - Math.pow(R, 2) - 2 * a2 - (4 * a3 * a2 - 8 * a1 - Math.pow(a3, 3)) / (4 * R))
        ;

        return new double[] {
            -a3 / 4 + R / 2 + D / 2,
            -a3 / 4 + R / 2 - D / 2,
            -a3 / 4 - R / 2 + E / 2,
            -a3 / 4 - R / 2 - E / 2
        };
    }

    public static void main(String[] args) {
        System.out.println(calcConstantVelocity(12.4, 9, 8, 4, 5, -5, 3.5, 9.8));
    }

}
