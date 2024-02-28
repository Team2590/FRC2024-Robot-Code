package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Superstructure;
import frc.robot.subsystems.drive.Drive;
import java.util.function.Function;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class AutoRoutines {

  public static final LoggedDashboardChooser<Command> buildChooser(
      Drive drive, Superstructure superstructure) {
    LoggedDashboardChooser<Command> autoChooser = new LoggedDashboardChooser<>("Auto Choices");

    PathPlannerPaths paths = PathPlannerPaths.create();
    AutoFunction ezAuto = (dynamicPaths) -> buildAuto(paths, drive, superstructure, dynamicPaths);

    // Register all the auto routines here
    autoChooser.addOption("OneNoteAuto", oneNoteAuto(paths, drive, superstructure));
    // This is the same as OneNoteAuto
    autoChooser.addOption(
        "EasyTwoNoteAuto", ezAuto.apply("shoot", "startB_note1", "shoot", "n2-n3", "snap_shoot"));

    // dispose of the paths, unused paths with be garbage collected.
    paths.dispose();
    return autoChooser;
  }

  /** Creates a single note auto. */
  private static Command oneNoteAuto(
      PathPlannerPaths paths, Drive drive, Superstructure superstructure) {
    return new AutoCommandBuilder(paths, drive, superstructure)
        .shoot(false)
        .startPath("startB_note1")
        .shoot(false)
        .followPath("n2-n3")
        .shoot(true)
        .build();
  }

  /**
   * Easy way to configure an Auto Routine, just pass in the paths.
   *
   * <p>Use 'shoot' for just shooting or 'snap_shoot' for snaping to target and shooting or 'intake'
   * for running intake.
   */
  private static Command buildAuto(
      PathPlannerPaths pathPlans, Drive drive, Superstructure superstructure, String... paths) {
    AutoCommandBuilder builder = new AutoCommandBuilder(pathPlans, drive, superstructure);
    for (String path : paths) {
      switch (path) {
        case "shoot":
          builder.shoot(false);
          break;
        case "snap_shoot":
          builder.shoot(true);
          break;
        case "intake":
          // intake automatically starts up after shoot but just in case if we need it.
          builder.intake();
          break;
        default:
          // if this was to be the first path, AutoCommandBuilder will make it the StartPathCommand.
          builder.followPath(path);
          break;
      }
    }
    return builder.build();
  }

  @FunctionalInterface
  interface AutoFunction extends Function<String[], Command> {
    @Override
    Command apply(String... args);
  }
}
