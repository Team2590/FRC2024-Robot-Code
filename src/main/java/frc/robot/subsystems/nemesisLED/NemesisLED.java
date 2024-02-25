package frc.robot.subsystems.nemesisLED;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.simulation.AddressableLEDSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

/**
 * Controls the LEDs. States: OFF, SOLID, FLASH (between white and other), ALTERNATE, CYCLE,
 * NEMESIS, and CELEBRATE
 *
 * @author Jeevan Sailesh, Elan Ronen, Shashank Madala
 * @see <a
 *     href="https://docs.wpilib.org/en/stable/docs/software/hardware-apis/misc/addressable-leds.html">WPILIB
 *     LED Tutorial</a>
 */
public class NemesisLED extends SubsystemBase {

  private static NemesisLED instance = null;

  public static NemesisLED getInstance() {
    return instance == null
        ? instance = new NemesisLED(LEDConstants.LED_PORT, LEDConstants.BUFFER_LENGTH)
        : instance;
  }

  private final AddressableLED led;
  private final AddressableLEDBuffer ledBuffer;
  private final int length;
  private final AddressableLEDSim ledSim;

  private int r, g, b;
  private double timer;
  private int[][] altRGBs;
  private int startLED;
  private int firstHue;

  private States state = States.OFF;

  private enum States {
    OFF,
    SOLID,
    FLASH,
    ALTERNATE,
    CYCLE,
    NEMESIS,
    CELEBRATE,
    INTAKE,
    HAS_NOTE,
    SCORE_AMP,
    IDLE,
    DISABLED,
  }

  /**
   * New NemesisLED object
   *
   * @param port - PWM port the LEDs are connected to
   * @param bufferLength - number of LEDs
   */
  public NemesisLED(int port, int bufferLength) {
    led = new AddressableLED(port);
    ledBuffer = new AddressableLEDBuffer(bufferLength);
    length = ledBuffer.getLength();
    led.setLength(length);
    led.setData(ledBuffer);
    led.start();
    ledSim = new AddressableLEDSim(led);
  }

  // @Override
  // public void periodic() {
  //     switch (state) {
  //         case OFF:
  //             solid(0, 0, 0);
  //             break;
  //         case SOLID:
  //             solid(r, b, g);
  //             break;
  //         case FLASH:
  //         System.out.println("in flash state");
  //             flash(r, g, b);
  //             timer += LEDConstants.REFRESH_RATE;
  //             break;
  //         case ALTERNATE:
  //             alternate(altRGBs);
  //             timer += LEDConstants.REFRESH_RATE;
  //             break;
  //         case CYCLE:
  //             cycle(altRGBs);
  //             break;
  //         case NEMESIS:
  //             nemesis();
  //             timer += LEDConstants.REFRESH_RATE;
  //             break;
  //         case CELEBRATE:
  //             CELEBRATE();
  //             break;
  //     }
  // }

  /** Turns the LEDs off. */
  // public void setOff() {
  //     state = States.OFF;
  // }

  // /**
  //  * Sets the LEDs to a constant solid color.
  //  * @param r - red RGB value
  //  * @param g - green RGB value
  //  * @param b - blue RGB value
  //  */
  // public void setSolid(int r, int g, int b) {
  //     this.r = r;
  //     this.g = g;
  //     this.b = b;
  //     state = States.SOLID;
  // }

  // /**
  //  * Sets the LEDs to a constant solid color.
  //  * Uses the last color given.
  //  */
  // public void setSolid() {
  //     state = States.SOLID;
  // }

  // /**
  //  * Flahes the LEDs between the color and white.
  //  * Each color is shown for {@link frc.settings.LEDsettings#FLASH_INTERVAL FLASH_INTERVAL}
  // milliseconds.
  //  * @param r - red RGB value
  //  * @param g - green RGB value
  //  * @param b - blue RGB value
  //  */
  // public void setFlash(int r, int g, int b) {
  //     this.r = r;
  //     this.g = g;
  //     this.b = b;
  //     timer = 0;
  //     state = States.FLASH;
  // }

  // /**
  //  * Flahes the LEDs between the color and white.
  //  * Uses the last color given.
  //  * Each color is shown for {@link frc.settings.LEDsettings#FLASH_INTERVAL FLASH_INTERVAL}
  // milliseconds.
  //  */
  // public void setFlash() {
  //     timer = 0;
  //     state = States.FLASH;
  // }

  // /**
  //  * Flashes the LEDs between the RGBs.
  //  * Each color is shown for {@link frc.settings.LEDsettings#FLASH_INTERVAL FLASH_INTERVAL}
  // milliseconds.
  //  * @param rgbs
  //  */
  // public void setAlternate(int[]... rgbs) {
  //     altRGBs = rgbs;
  //     timer = 0;
  //     state = States.ALTERNATE;
  // }

  // /**
  //  * Flashes the LEDs between the RGBs.
  //  * Uses the last RGBs given.
  //  * Each color is shown for {@link frc.settings.LEDsettings#FLASH_INTERVAL FLASH_INTERVAL}
  // milliseconds.
  //  */
  // public void setAlternate() {
  //     timer = 0;
  //     state = States.ALTERNATE;
  // }

  // /**
  //  * Cycles the LEDs around the strip between the RGBs.
  //  * @param rgbs
  //  */
  // public void setCycle(int[]... rgbs) {
  //     altRGBs = rgbs;
  //     state = States.CYCLE;
  // }

  // /**
  //  * Cycles the LEDs around the strip between the RGBs.
  //  * Uses the last RGBs given.
  //  */
  // public void setCycle() {
  //     state = States.CYCLE;
  // }

  // /**
  //  * Hoist the Colors
  //  */
  // public void setNemesis() {
  //     state = States.NEMESIS;
  // }

  // /**
  //  * CELEBRATE MODE!
  //  */
  // public void setCELEBRATE() {
  //     state = States.CELEBRATE;
  // }

  public void solid(int r, int b, int g) {
    for (int i = 0; i < length; i++) {
      ledBuffer.setRGB(i, r, b, g);
    }
    led.setData(ledBuffer);
  }

  public void flash(int r, int g, int b) {
    if (timer >= 2 * LEDConstants.FLASH_INTERVAL) {
      timer = 0;
    }

    if (timer < LEDConstants.FLASH_INTERVAL) {
      solid(255, 255, 255);
    } else {
      solid(r, b, g);
    }
  }

  public void alternate(int[]... rgbs) {
    if (timer >= LEDConstants.FLASH_INTERVAL) timer = 0;

    final var rgb = rgbs[(int) (rgbs.length * timer / LEDConstants.FLASH_INTERVAL)];
    solid(rgb[0], rgb[1], rgb[2]);
  }

  private void cycle(int[]... rgbs) {
    int[] rgb;
    int i = startLED;
    int n = 1;

    do {
      rgb = rgbs[n * rgbs.length / length];
      ledBuffer.setRGB(i, rgb[0], rgb[1], rgb[2]);

      i = (i + 1) % length;
      n++;
    } while (i == startLED);

    startLED = (startLED + 1) % length;
  }

  public void nemesis() {
    cycle(new int[] {255, 255, 255}, new int[] {255, 0, 0});
  }

  public void CELEBRATE() {
    for (int i = 0; i < length; i++) {
      final var hue = (firstHue + (i * 180 / length)) % 180;
      ledBuffer.setHSV(i, hue, 255, 208);
    }
    firstHue += 1;
    firstHue %= 180;

    led.setData(ledBuffer);
  }
}
