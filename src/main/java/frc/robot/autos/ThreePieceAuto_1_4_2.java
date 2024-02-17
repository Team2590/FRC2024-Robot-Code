package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Superstructure;
import frc.robot.subsystems.drive.Drive;

/** A three piece auto routine that picks up notes 1, 4, and 2 */
public class ThreePieceAuto_1_4_2 extends SequentialCommandGroup {

  public ThreePieceAuto_1_4_2(PathPlannerPaths paths, Drive drive, Superstructure superstructure) {
    addRequirements(drive);

    addCommands(
        // TODO: command to start the start the intake system.
        // Move to note1 from starting position B (speaker)
        paths.getFollowPathCommand("startB_note1"),
        // Intake
        new InstantCommand(() -> superstructure.intake()),
        // TODO: Position the robote towards speaker??
        // Could we use snapping logic to lock on to target.
        // Shoot
        new InstantCommand(() -> superstructure.shoot()));
  }
}
