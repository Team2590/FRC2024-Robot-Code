package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
        // TODO: Aim the robote towards speaker??
        // Could we use snapping logic to lock on to target.
        // Shoot
        Commands.print("Shoot Note1"),
        new InstantCommand(() -> superstructure.shoot()),
        paths.getFollowPathCommand("note1_note4"));
  }
}