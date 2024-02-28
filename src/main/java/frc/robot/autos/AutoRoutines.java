package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Superstructure;
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class AutoRoutines {

  public static final LoggedDashboardChooser<Command> buildChooser(
      Drive drive, Superstructure superstructure) {
    LoggedDashboardChooser<Command> autoChooser = new LoggedDashboardChooser<>("Auto Choices");

    PathPlannerPaths paths = PathPlannerPaths.create();
    // Register all the auto routines here
    autoChooser.addOption("OneNoteAuto", oneNoteAuto(paths, drive, superstructure));
    autoChooser.addOption(
        "EasyTwoNoteAuto",
        buildAuto(paths, drive, superstructure, "shoot", "startB_note1", "snap_shoot", "n2-n3"));

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
   * Easy way to configure an Auto Routine, just path in the paths.
   *
   * <p>Use 'shoot' for just shooting or 'snap_shoot' for snaping to target and shooting.
   */
  private static Command buildAuto(
      PathPlannerPaths pathPlans, Drive drive, Superstructure superstructure, String... paths) {
    AutoCommandBuilder builder = new AutoCommandBuilder(pathPlans, drive, superstructure);
    for (String path : paths) {
      if ("shoot".equals(path)) {
        builder.shoot(false);
      } else if ("snap_shoot".equals(path)) {
        builder.shoot(true);
      } else {
        // if this was to be the first path, AutoCommandBuilder will make it the StartPathCommand.
        builder.followPath(path);
      }
    }
    return builder.build();
  }
}
