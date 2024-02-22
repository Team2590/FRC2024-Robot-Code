package frc.robot.util;


import edu.wpi.first.wpilibj.Joystick;

/**
 * A wrapper around {@link Joystick} that deadbands the x and y values from a joystick. Note that
 * {@link #getRawButtonPressed(int)} and {@link #getRawButtonReleased(int)} should only be called
 * once each at most per loop iteration.
 *
 * @author Elan Ronen
 * @see <a
 *     href="https://docs.wpilib.org/en/stable/docs/software/basic-programming/joystick.html">WPILIB
 *     Joystick Tutorial</a>
 * @see <a
 *     href="https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj/Joystick.html">WPILIB
 *     Joystick Docs</a>
 * @see <a href="https://www.desmos.com/calculator/8m5abgxpit">Desmos visual</a>
 */
public class BandedJoystick extends Joystick {

  private final double DEADBAND_X;
  private final double DEADBAND_Y;
  private final double DEADBAND_X_CONST;
  private final double DEADBAND_Y_CONST;

  /**
   * Makes a new Joystick
   *
   * @param port - port the joystick is on
   * @param xDeadBand - the deadband for the x axis [0, 1)
   * @param yDeadBand - the deadband for the y axis [0, 1)
   */
  public BandedJoystick(int port, double deadbandX, double deadbandY) {
    super(port);
    DEADBAND_X = deadbandX;
    DEADBAND_Y = deadbandY;
    DEADBAND_X_CONST = 1 / (1 - deadbandX);
    DEADBAND_Y_CONST = 1 / (1 - deadbandY);
  }

  /**
   * Gets the banded value of the joystick's x axis
   *
   * <p>Formula: D * (x - s) + s {|x| > d} where D = 1 / (1 - deadband), s = signum(x)
   *
   * @return banded val of the joystick's x axis
   */
  public double getXBanded() {
    final double x = getX();
    if (Math.abs(x) < DEADBAND_X) return 0;

    final double s = Math.signum(x);
    return DEADBAND_X_CONST * (x - s) + s;
  }

  /**
   * Gets the banded value of the joystick's y axis
   *
   * <p>Formula: D * (y - s) + s {|y| > d} where D = 1 / (1 - deadband), s = signum(y)
   *
   * @return banded val of the joystick's y axis
   */
  public double getYBanded() {
    final double y = getY();
    if (Math.abs(y) < DEADBAND_Y) return 0;

    final double s = Math.signum(y);
    return DEADBAND_Y_CONST * (y - s) + s;
  }
}
