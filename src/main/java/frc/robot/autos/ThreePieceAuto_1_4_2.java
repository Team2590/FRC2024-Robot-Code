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
        // Start up the intake system and follow path to first position in parallel.
        Commands.parallel(
            // TODO: command to start the start the intake system.
            Commands.print("Starting up Intake .... "),
            // Move to note1 from starting position B (speaker)
            paths.getFollowPathCommand("startB_note1")),
        Commands.print("Picking up Note1 "),
        // Intake
        new InstantCommand(() -> superstructure.intake()),
        // TODO: Need methods like following to wait for the robot
        // to get into the right state before executing actions?
        // Commands.waitUntil(superstructure.isReadyToShoot()),

        // TODO: Position the robote towards speaker??
        // Could we use snapping logic to lock on to target.
        // Shoot
        Commands.print("Shoot Note1"),
        new InstantCommand(() -> superstructure.shoot()));
  }
}
