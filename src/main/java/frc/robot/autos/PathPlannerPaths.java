package frc.robot.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import java.io.File;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;
import java.util.stream.Stream;

public class PathPlannerPaths {

  private final Map<String, PathPlannerPath> paths;

  private PathPlannerPaths(Map<String, PathPlannerPath> paths) {
    this.paths = paths;
  }

  public static PathPlannerPaths create() {
    // Loads up all the paths automatically.
    Map<String, PathPlannerPath> allPaths = loadAllPaths();
    // add additional path if needed into allpaths.
    return new PathPlannerPaths(allPaths);
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

  /**
   * Automatically loads up all the path files under deploy directory.
   *
   * @return
   */
  private static Map<String, PathPlannerPath> loadAllPaths() {
    Map<String, PathPlannerPath> paths = new HashMap<>();
    List<String> pathFileNames = getAllPathFileNames();

    if (pathFileNames.isEmpty()) {
      System.err.println("Error: No path files found");
    }
    for (String pathFile : pathFileNames) {
      addPath(paths, pathFile);
    }
    return paths;
  }

  public static List<String> getAllPathFileNames() {
    File[] pathFiles = new File(Filesystem.getDeployDirectory(), "pathplanner/paths").listFiles();

    if (pathFiles == null) {
      return new ArrayList<>();
    }

    return Stream.of(pathFiles)
        .filter(file -> !file.isDirectory())
        .map(File::getName)
        .filter(name -> name.endsWith(".path"))
        .map(name -> name.substring(0, name.lastIndexOf(".")))
        .collect(Collectors.toList());
  }
}
