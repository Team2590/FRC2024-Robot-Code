package frc.robot.subsystems.nemesisLED;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Controls the LEDs on the robot.
 *
 * @author Dhruv Shah
 * @author Elan Ronen
 * @author Shashank Madala
 * @author Ian Keller
 * @see <a href=
 *     "https://docs.wpilib.org/en/stable/docs/software/hardware-apis/misc/addressable-leds.html">WPILIB
 *     LED Tutorial</a>
 */
public class NemesisLED extends SubsystemBase {
  private final AddressableLED led;
  private final AddressableLEDBuffer ledBuffer;
  private final int length;
  private int hue = 0;
  private int timer;

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
  }

  /**
   * Sets the leds to a solid color of the given rgb value
   * @param r - value of red
   * @param g - value of green
   * @param b - value of blue
   */
  public void setRGB(int r, int g, int b) {
    System.out.println("This is running");
    for (int i = 0; i < length; i++) {
      ledBuffer.setRGB(i, g, r, b);
    }
    led.setData(ledBuffer);
  }

  /**
   * Sets the leds to a flashing color of the given rgb value at the given rate
   * @param r - value of red
   * @param g - value of green
   * @param b - value of blue
   * @param flashInterval - milliseconds between color changes
   */
  public void setFlashing(int r, int g, int b, int flashInterval) {
    timer += 20;
    if (timer >= flashInterval * 2) { timer = 0; }
    if (timer < flashInterval ) { setRGB(r, g, b); }
    else { off(); }
  }

  /**
   * Sets the leds to flashing 2 alternating colors of the given rgb values at the given rate
   * @param r1 - value of red of 1st color
   * @param g1 - value of green of 1st color
   * @param b1 - value of blue of 1st color
   * @param r2 - value of red of 2nd color
   * @param g2 - value of green of 2nd color
   * @param b2 - value of blue of 2nd color
   * @param flashInterval - milliseconds between color changes
   */
  public void setAlternating(int r1, int g1, int b1, int r2, int g2, int b2, int flashInterval) {
    timer += 20;
    if (timer >= flashInterval * 2) { timer = 0; }
    if (timer < flashInterval ) { setRGB(r1, g1, b1); }
    else { setRGB(r2, g2, b2); }
  }

  /** Set the leds to a static rainbow */
  public void setRainbow() {
    int firstPixelHue = 0;
    for (var i = 0; i < ledBuffer.getLength(); i++) {
      final var rainbowHue = (firstPixelHue + (i * 180 / ledBuffer.getLength())) % 180;
      ledBuffer.setHSV(i, rainbowHue, 255, 128);
    }
    firstPixelHue += 3;
    firstPixelHue %= 180;
    led.setData(ledBuffer);
  }

  /** Set the leds to a breathing rainbow */
  public void setBreathing() {
    for (var i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setHSV(i, hue, 255, 128);
    }
    hue += 1;
    led.setData(ledBuffer);
  }

  /** Set the leds to a flowing rainbow */
  public void setFlow() {
    for (var i = 0; i < ledBuffer.getLength(); i++) {
      final var rainbowHue = (hue + (i * 180 / ledBuffer.getLength())) % 180;
      ledBuffer.setHSV(i, rainbowHue, 255, 128);
    }
    hue += 3;
    hue %= 180;
    led.setData(ledBuffer);
  }

  /** Set the leds to a candy cane styyle */
  public void setCandyCane() {
    for (var i = 0; i < ledBuffer.getLength(); i++) {
      if (i % 2 == 0) {
        ledBuffer.setRGB(i, 0, 255, 0);
      } else {
        ledBuffer.setRGB(i, 255, 255, 255);
      }
    }
    led.setData(ledBuffer);
  }

  /** A constant color for the leds */
  public enum Color {
    Red(255, 0, 0),
    Orange(255, 165, 0),
    Yellow(255, 255, 0),
    Green(0, 255, 0),
    Blue(0, 0, 255),
    Violet(238, 138, 238);

    public final int r;
    public final int g;
    public final int b;

    private Color(int r, int g, int b) {
      this.r = r;
      this.g = g;
      this.b = b;
    }
  }

  /**
   * Set the leds to one of the color constants
   * @param color
   */
  public void setColor(Color color) {
    setRGB(color.r, color.g, color.b);
  }

  private int t = 0;
  private final int buffer = 10;

  /** Set the leds to a flowing candy cane style */
  public void setCandyCaneFlow() {
    t += 0.2;
    for (var i = 0; i < ledBuffer.getLength(); i++) {
      if ((i + t) % buffer > buffer / 2) {
        ledBuffer.setRGB(i, 0, 255, 0);
      } else {
        ledBuffer.setRGB(i, 255, 255, 255);
      }
    }
    led.setData(ledBuffer);
  }

  /**
   * Hoist the colors. 
   * <p>A gradient between red and either white or black
   * @param secondaryWhite - secondary color; true for white, false for black
   */
  public void setNemesis(boolean secondaryWhite) {
    // white secondary
    if (secondaryWhite) {
      for (int i = 0; i < ledBuffer.getLength(); i++) {
        int hue = 255 / (Math.abs((i % 20) - 10));
        ledBuffer.setRGB(i, 255, hue, hue);
      }
    // black secondary
    } else {
      for (int i = 0; i < ledBuffer.getLength(); i++) {
        int hue = 255 / (Math.abs((i % 20) - 10));
        ledBuffer.setRGB(i, hue, 0, 0);
      }
    }
    led.setData(ledBuffer);
  }

  /** Turn off the leds */
  public void off() {
    for (int i = 0; i < length; i++) {
      ledBuffer.setRGB(i, 0, 0, 0);
    }
    led.setData(ledBuffer);
  }
}
