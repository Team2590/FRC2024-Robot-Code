package frc.robot.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.HashMap;
import java.util.Map;

public class PathPlannerPaths {

  private final Map<String, Command> paths;

  private PathPlannerPaths(Map<String, Command> paths) {
    this.paths = paths;
  }

  public static PathPlannerPaths create() {
    Map<String, Command> paths = new HashMap<>();
    addPath(paths, "startB_note1");
    addPath(paths, "note1_note4");

    return new PathPlannerPaths(paths);
  }

  /** Command to execute path for the given pathName. Throws an exception if the */
  public Command getFollowPathCommand(String pathName) {
    Command command = paths.get(pathName);
    if (command == null) {
      throw new RuntimeException("Can't find command for " + pathName);
    }
    return command;
  }

  public void dispose() {
    paths.clear();
  }

  private static void addPath(Map<String, Command> paths, String pathName) {
    PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
    paths.put(pathName, AutoBuilder.followPath(path));
  }
}
