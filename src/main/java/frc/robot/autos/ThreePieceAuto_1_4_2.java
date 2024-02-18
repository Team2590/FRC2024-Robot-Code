package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Superstructure;
import frc.robot.subsystems.drive.Drive;

/** A three piece auto routine that picks up notes 1, 4, and 2 */
public class ThreePieceAuto_1_4_2 extends SequentialCommandGroup {

  public ThreePieceAuto_1_4_2(PathPlannerPaths paths, Drive drive, Superstructure superstructure) {
    addRequirements(drive);
    addCommands(
        new StartPathCommand(paths, "startB_note1"),
        Commands.print("Picking up Note1 "),
        // Intake
        new InstantCommand(() -> superstructure.intake()),
        // TODO: Need methods to wait for the robot
        // to get into the right state before executing actions?
        // Commands.waitUntil(superstructure.isReadyToShoot()),

        // TODO: Aim the robote towards speaker??
        // Could we use snapping logic to lock on to target.
        // Shoot
        Commands.print("Shoot Note1"),
        new InstantCommand(() -> superstructure.shoot()),
        paths.getFollowPathCommand("note1_note4"));
    // TODO: intake note4, shoot and then go to note 2, intake, move to speaker and shoot.
  }
}
