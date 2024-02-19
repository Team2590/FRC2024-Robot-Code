package frc.robot.util;

/**
 * @author: not griffin
 * @author: Rohan & Abhik
 */
public class LookupTable {
  private double[] keys;
  private double[] values;

  public LookupTable(double[] keys, double[] values) {
    this.keys = keys;
    this.values = values;
  }

  // map format (distanceToTarget, RPM)
  public double getValue(double distance) {
    double tempLow = 0;
    int lowerIndex = 0;
    int upperIndex = 0;

    int arrSize = keys.length;

    if (distance > keys[arrSize - 1]) {
      return values[arrSize - 1];
    } else {
      for (int i = 1; i <= keys.length - 1; i++) {
        tempLow = keys[i];
        if (tempLow > distance) {
          lowerIndex = i - 1;
          upperIndex = i;
          break;
        }
      }
    }
    double deltaSpeed = values[upperIndex] - values[lowerIndex];
    double slopeSpeed = deltaSpeed / (keys[upperIndex] - keys[lowerIndex]);
    double ret = slopeSpeed * (distance - keys[lowerIndex]) + values[lowerIndex];

    // System.out.println("returning this speed: " + ret);
    return ret;
  }
}

/***
 * easter egg
 * .._____/\__/\_/\
 * _/ (o)......... \___/|
 * \vv| / / /....... __ |
 * \ ________________/ \|
 *
 */