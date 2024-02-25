package frc.robot.autos;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Superstructure;
import frc.robot.subsystems.drive.Drive;

public class AutoRoutines {

  public static final LoggedDashboardChooser<Command> buildChooser(
      Drive drive, Superstructure superstructure) {
    LoggedDashboardChooser<Command> autoChooser = new LoggedDashboardChooser<>("Auto Choices");

    PathPlannerPaths paths = PathPlannerPaths.create();
    // Register all the auto routines here
    autoChooser.addOption("OneNoteAuto", oneNoteAuto(paths, drive, superstructure));

    // dispose of the paths, unused paths with be garbage collected.
    paths.dispose();
    return autoChooser;
  }

  /** Creates a single note auto. */
  private static Command oneNoteAuto(
      PathPlannerPaths paths, Drive drive, Superstructure superstructure) {
    return new AutoCommandBuilder(paths, drive, superstructure)
        .startPath("startB_note1")
        .shoot(false)
        .build();
  }
}
