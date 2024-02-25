package frc.robot.autos;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

public class PathPlannerPaths {

  private final Map<String, PathPlannerPath> paths;
  private final List<PathPlannerPath> _paths;

  private PathPlannerPaths(Map<String, PathPlannerPath> paths) {
    this.paths = paths;
    this._paths = null;
  }

  private PathPlannerPaths(List<PathPlannerPath> paths) {

    this.paths = null;
    this._paths = paths;
  }

  public static PathPlannerPaths create() {
    Map<String, PathPlannerPath> paths = new HashMap<>();

    // Add paths that will be used in Auto routines.
    addPath(paths, "startB_note1");
    // addPath(paths, "note1_note4");

    return new PathPlannerPaths(paths);
  }

  /**
   * Command to execute path for the given pathName. Throws an exception if the path is not found.
   */
  public Command getFollowPathCommand(String pathName) {
    return AutoBuilder.followPath(getPath(pathName));
  }

  /** Returns the starting pose from the given path name. */
  public Pose2d getStartingPose(String pathName) {
    return getPath(pathName).getPreviewStartingHolonomicPose();
  }

  public static List<PathPlannerPath> get_path_group(String name) {

    return PathPlannerAuto.getPathGroupFromAutoFile(name);
  }

  public void dispose() {
    paths.clear();
  }

  private PathPlannerPath getPath(String pathName) {
    PathPlannerPath path = paths.get(pathName);
    if (path == null) {
      throw new RuntimeException("Can't find path for " + pathName);
    }
    return path;
  }

  private static void addPath(Map<String, PathPlannerPath> paths, String pathName) {
    PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
    paths.put(pathName, path);
  }

  // private static void addPathViaAuto(Map<String, PathPlannerPath> paths, String Auto_name) {
  //   List<PathPlannerPath> p = PathPlannerAuto.getPathGroupFromAutoFile(Auto_name);
  //   String name = Auto_name.replaceAll("\\.jpeg$", "");

  //   for (int i = 0; i < p.size(); i++) {

  //     paths.put(name + "_" + i, p.get(i));
  //   }
  // }
}
