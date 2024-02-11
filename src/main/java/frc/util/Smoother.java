package frc.util;
//
import java.util.function.Supplier;

/**
 * Smooths values over time using averages. Runs in constant time.
 *
 * @author Elan Ronen
 */
public class Smoother {

  /** Number of measurements to store */
  public final int size;

  private final double[] values; // Queue
  private double sum; // Sum of numbers in the queue
  private int items; // Number of items in the queue
  private int i = -1; // Index of the most recent item
  private double average; // The average
  private double lastAverage; // Last average (does not reset)

  /**
   * The constructor
   *
   * @param n - the number of values to remember
   */
  public Smoother(int n) {
    size = n;
    values = new double[size];
  }

  /**
   * Wraps a method to automatically {@link Wrapper#update() update()} data from and {@link
   * #push(double)} it into a {@link Smoother}.
   *
   * @param n - the number of values to remember
   * @param getter - the method to get numbers from
   * @return a {@link Smoother} {@link Wrapper} around the getter
   */
  public static Wrapper wrap(int n, Supplier<Double> getter) {
    return new Wrapper(n, getter);
  }

  /**
   * Add a value (and delete the oldest if more than {@link #size} values)
   *
   * @param v - the value to add
   * @return the average
   */
  public double push(double v) {
    i = (i + 1) % size;
    sum -= values[i];
    sum += v;
    values[i] = v;

    if (items != size) items++;

    lastAverage = average = sum / items;
    return average;
  }

  /**
   * Get the average
   *
   * @return the average
   */
  public double get() {
    return average;
  }

  /**
   * Get the last average (does not reset with {@link #reset()})
   *
   * @return the last average
   */
  public double getLast() {
    return lastAverage;
  }

  /** Reset to initial state */
  public void reset() {
    if (i == -1) return;

    for (int i = 0; i < size; i++) {
      values[i] = 0;
    }
    sum = 0;
    items = 0;
    i = -1;
    average = 0;
  }

  /**
   * Wraps a method to automatically {@link #update()} data from and {@link #push(double)} it into a
   * {@link Smoother}.
   */
  public static class Wrapper extends Smoother {

    private final Supplier<Double> getter;

    /**
     * The constructor
     *
     * @param n - the number of values to remember
     * @param getter - the method to get numbers from
     */
    public Wrapper(int n, Supplier<Double> getter) {
      super(n);
      this.getter = getter;
    }

    /**
     * Add a value fetched from the getter (and delete the oldest if more than {@link Smoother#size
     * size} values)
     */
    public void update() {
      push(getter.get());
    }
  }
}
