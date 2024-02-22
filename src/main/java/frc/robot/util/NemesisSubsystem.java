package frc.robot.util;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.ArrayList;
import java.util.function.Supplier;

/**
 * A base class for any class that has an update method and has data to be published to the
 * dashboard.
 *
 * @author Elan Ronen
 */
public abstract class NemesisSubsystem {

  /** Updates over a regular interval. This method should be called once per loop iteration. */
  public abstract void update();

  /** List of functions that get data from a getter and publish it to an entry. */
  private final ArrayList<Runnable> shuffleboardPublishers = new ArrayList<>();

  /**
   * Adds an entry that will automatically be published to by {@link #outputShuffleboard()} with
   * data fetched from the getter.
   *
   * @param key - the string name of the shuffleboard field
   * @param getter - the getter method to get the data from
   * @param defaultValue - a default value in case the getter returns null (do not make this null)
   */
  protected <T> void addSmartDashboardEntry(String key, Supplier<T> getter, T defaultValue) {
    if (defaultValue == null)
      throw new NullPointerException("addSmartDashboardEntry defaultValue argument cannot be null");
    final var entry = SmartDashboard.getEntry(key);
    shuffleboardPublishers.add(
        () -> {
          final var value = getter.get();
          entry.setValue(value == null ? defaultValue : value);
        });
  }

  /**
   * Outputs data from the getters to the entries with the names from {@link
   * #addSmartDashboardEntry()}. Can be overriden for custom fuctionality, but will make {@link
   * #addSmartDashboardEntry(String, Supplier, Object)} obsolete.
   */
  public void outputShuffleboard() {
    shuffleboardPublishers.forEach(runnable -> runnable.run());
  }

  /**
   * Outputs the data from given subsytems to the suffleboard by calling their {@link
   * #outputShuffleboard()} methods.
   *
   * @param subsystems - the subsytems to output the data of
   */
  public static final void outputShuffleboardOf(NemesisSubsystem... subsystems) {
    for (final var subsystem : subsystems) {
      subsystem.outputShuffleboard();
    }
  }
}
