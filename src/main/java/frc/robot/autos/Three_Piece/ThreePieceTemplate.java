package frc.robot.autos.Three_Piece;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Superstructure;
import frc.robot.autos.PathPlannerPaths;
import frc.robot.autos.StartPathCommand;
import frc.robot.subsystems.drive.Drive;
import java.util.List;

public class ThreePieceTemplate extends SequentialCommandGroup {

  private List<PathPlannerPath> paths;

  public ThreePieceTemplate(String auto_name, Superstructure superstructure, Drive drive) {

    paths = PathPlannerPaths.get_path_group(auto_name);
    addRequirements(drive);
    addCommands(
        new InstantCommand(() -> superstructure.shoot()),
        Commands.parallel(
            new StartPathCommand(paths.get(0)),
            new InstantCommand(() -> superstructure.intake()),
            new InstantCommand(() -> superstructure.speedShooter())),
        new InstantCommand(() -> superstructure.shoot()),
        Commands.parallel(
            AutoBuilder.followPath(paths.get(1)),
            new InstantCommand(() -> superstructure.intake())),
        Commands.parallel(
            new InstantCommand(() -> superstructure.speedShooter()),
            AutoBuilder.followPath(paths.get(2))),
        new InstantCommand(() -> superstructure.shoot()));
  }
}
