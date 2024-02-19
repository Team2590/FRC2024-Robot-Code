package frc.util;

/** Misc. helper functions. */
public class HelperFn {

  // Helper class with static functions, not instantiable.
  private HelperFn() {}

  /**
   * Returns true if the currentValue is within +/- of tolerance of the setPoint.
   *
   * @param currentValue
   * @param setPoint
   * @param tolerance
   * @return
   */
  public static boolean isWithinTolerance(double currentValue, double setPoint, double tolerance) {
    // Check if tolerance is positive
    if (tolerance <= 0) {
      throw new IllegalArgumentException("Tolerance must be positive.");
    }

    // Calculate the lower and upper bounds of the tolerance range
    double lowerBound = setPoint - tolerance;
    double upperBound = setPoint + tolerance;

    // Check if currentValue is within the bounds
    return currentValue >= lowerBound && currentValue <= upperBound;
  }
}
