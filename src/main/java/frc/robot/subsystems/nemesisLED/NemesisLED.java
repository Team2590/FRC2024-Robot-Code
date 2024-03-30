package frc.robot.subsystems.nemesisLED;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

/**
 * Controls the LEDs. States: OFF, SOLID, FLASH (between white and other), ALTERNATE, CYCLE,
 * NEMESIS, and CELEBRATE
 *
 * @author Dhruv Shah, Elan Ronen, Shashank Madala
 * @see <a href=
 *     "https://docs.wpilib.org/en/stable/docs/software/hardware-apis/misc/addressable-leds.html">WPILIB
 *     LED Tutorial</a>
 */
public class NemesisLED extends SubsystemBase {
  private final AddressableLED led;
  private final AddressableLEDBuffer ledBuffer;
  private final int length;
  private int hue = 0;

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

  public void setRGB(int r, int g, int b) {
    for (int i = 0; i < length; i++) {
      ledBuffer.setRGB(i, g, r, b);
    }
    led.setData(ledBuffer);
  }

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

  public void setBreathing() {
    for (var i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setHSV(i, hue, 255, 128);
    }
    hue += 1;
    led.setData(ledBuffer);
  }

  public void setFlow() {
    for (var i = 0; i < ledBuffer.getLength(); i++) {
      final var rainbowHue = (hue + (i * 180 / ledBuffer.getLength())) % 180;
      ledBuffer.setHSV(i, rainbowHue, 255, 128);
    }
    hue += 3;
    hue %= 180;
    led.setData(ledBuffer);
  }

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

  public void setColor(LEDConstants.Colors color) {
    setRGB(color.r, color.g, color.b);
  }

  private int candyCaneFlowTime = 0;
  private final int buffer = 10;

  public void setCandyCaneFlow() {
    candyCaneFlowTime += 0.2;
    for (var i = 0; i < ledBuffer.getLength(); i++) {
      if ((i + candyCaneFlowTime) % buffer > buffer / 2) {
        ledBuffer.setRGB(i, 0, 255, 0);
      } else {
        ledBuffer.setRGB(i, 255, 255, 255);
      }
    }
    led.setData(ledBuffer);
  }

  private int blinkingTime = 0;

  private int timesBlinked = 0;

  public void setBlinking(LEDConstants.Colors color) {
    if (blinkingTime % 5 == 0) {
      setColor(color);
      timesBlinked += 1;
    } else {
      off();
    }
    blinkingTime += 1;
  }

  // private boolean alreadyBlinking = false;
  // private double initalTime = 0;

  Timer timer = new Timer();

  // public void setBlinking(LEDConstants.Colors color, double time) {
  //   if (!alreadyBlinking) {
  //     alreadyBlinking = true;
  //     initalTime = timer.get();
  //   }
  //   else {
  //     if (timer.get() - initalTime >= time) {
  //       alreadyBlinking = false;
  //       off();
  //     } else {
  //       setBlinking(color);
  //     }
  //   }
  // }

  private int maxBlinks = 0;

  public void setBlinking(LEDConstants.Colors color, int blinks) {
    maxBlinks = blinks;
    if (maxBlinks % (blinks + 1) >= maxBlinks) {
      off();
    } else {
      setBlinking(color);
    }
  }

  public void off() {
    for (int i = 0; i < length; i++) {
      ledBuffer.setRGB(i, 0, 0, 0);
    }
    led.setData(ledBuffer);
  }
}
