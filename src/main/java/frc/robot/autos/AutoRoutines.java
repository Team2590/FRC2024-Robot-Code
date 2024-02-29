package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Superstructure;
import frc.robot.subsystems.drive.Drive;
import java.util.function.Function;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class AutoRoutines {

  private static final String SHOOT = "shoot";
  private static final String SNAP_SHOOT = "snap_shoot";
  private static final String INTAKE = "intake";

  public static final LoggedDashboardChooser<Command> buildChooser(
      Drive drive, Superstructure superstructure) {
    LoggedDashboardChooser<Command> autoChooser = new LoggedDashboardChooser<>("Auto Choices");

    PathPlannerPaths paths = PathPlannerPaths.create();
    // An easy way to create autos without having to supply paths, drive and superstructure for
    // every call.
    AutoFunction ezAuto = (instructions) -> buildAuto(paths, drive, superstructure, instructions);

    // Register all the auto routines here
    autoChooser.addOption("OneNoteAuto", oneNoteAuto(paths, drive, superstructure));
    // This is the same as OneNoteAuto
    autoChooser.addOption(
        "EasyTwoNoteAuto", ezAuto.apply(SHOOT, "startB_note1", SHOOT, "n2-n3", SNAP_SHOOT));

    autoChooser.addOption("startA_n1", ezAuto.apply("StartA", SNAP_SHOOT, "startA_n1", SNAP_SHOOT));

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
      PathPlannerPaths pathPlans,
      Drive drive,
      Superstructure superstructure,
      String... instructions) {
    AutoCommandBuilder builder = new AutoCommandBuilder(pathPlans, drive, superstructure);
    for (String path : instructions) {
      switch (path) {
        case SHOOT:
          builder.shoot(false);
          break;
        case SNAP_SHOOT:
          builder.shoot(true);
          break;
        case INTAKE:
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
