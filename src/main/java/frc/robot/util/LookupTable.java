package frc.robot.util;
/**
 * A class which stores a set of keys and corresponding values, and can return a value by 
 * comparing keys with given input when a value is requested.
 * @author not griffin
 * @author Rohan & Abhik
 * @author Ian Keller
 */
public class LookupTable {
  private double[] keys;
  private double[] values;

  /**
   * Create a new lookup table
   * <p>{@code keys} are the values that the lookup table will compare with your given value when a value is requested, 
   * while {@code values} are what gets returned. A common implementation of this class is for shooting, in which the 
   * {@code keys} are the tested distance from the target, and the {@code values} is the parameter (e.g. shooter RPM, 
   * shooter angle) that made a shot from that distance.
   * @param keys - the determining values
   * @param values - the values which are returned
   */
  public LookupTable(double[] keys, double[] values) {
    this.keys = keys;
    this.values = values;
  }

  /**
   * Returns the value for the given key. 
   * <p>If the key is not in the table, the value is estimated using the closest higher and lower keys.
   * @param comparator - the key to compare with
   * @return value for the given key
   */
  public double getValue(double comparator) {
    double tempLow = 0;
    int lowerIndex = 0;
    int upperIndex = 0;

    int arrSize = keys.length;

    if (comparator > keys[arrSize - 1]) {
      return values[arrSize - 1];
    } else {
      for (int i = 1; i <= keys.length - 1; i++) {
        tempLow = keys[i];
        if (tempLow > comparator) {
          lowerIndex = i - 1;
          upperIndex = i;
          break;
        }
      }
    }
    double deltaSpeed = values[upperIndex] - values[lowerIndex];
    double slopeSpeed = deltaSpeed / (keys[upperIndex] - keys[lowerIndex]);
    double ret = slopeSpeed * (comparator - keys[lowerIndex]) + values[lowerIndex];

    return ret;
  }
}
