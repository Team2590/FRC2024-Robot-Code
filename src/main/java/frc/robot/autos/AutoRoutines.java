package frc.robot.autos;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Superstructure;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.PhotonNoteRunnable;
import java.util.Optional;
import java.util.function.Function;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class AutoRoutines {

  private static final String SHOOT = "shoot";
  private static final String SNAP_SHOOT = "snap_shoot";

  public static final LoggedDashboardChooser<Command> buildChooser(
      Drive drive, Superstructure superstructure) {
    LoggedDashboardChooser<Command> autoChooser = new LoggedDashboardChooser<>("Auto Choices");

    PathPlannerPaths paths = PathPlannerPaths.create();
    // An easy way to create autos without having to supply paths, drive and superstructure for
    // every call.
    AutoFunction ezAuto = (instructions) -> buildAuto(paths, drive, superstructure, instructions);

    // Register all the auto routines here
    // autoChooser.addOption("OneNoteAuto", oneNoteAuto(paths, drive, superstructure));
    // This is the same as OneNoteAuto

    autoChooser.addOption(
        "2_startA_n1", ezAuto.apply("startA", SNAP_SHOOT, "startA_note1", SNAP_SHOOT));
    autoChooser.addOption(
        "2_startC_n3", ezAuto.apply("startC", SNAP_SHOOT, "startC_note3", SNAP_SHOOT));
    autoChooser.addOption("2_startB_n2", ezAuto.apply("startB", SHOOT, "startB_note2", SNAP_SHOOT));
    autoChooser.addOption(
        "3_startA_n1_n4",
        ezAuto.apply(
            "startA", SNAP_SHOOT, "startA_note1", SNAP_SHOOT, "note1_n4", "n4_return", SNAP_SHOOT));
    autoChooser.addOption(
        "3_startB_n2_n1",
        ezAuto.apply("startB", SHOOT, "startB_note2", SNAP_SHOOT, "note2_n1", SNAP_SHOOT));
    autoChooser.addOption(
        "3_startB_n2_n5",
        ezAuto.apply(
            "startB", SHOOT, "startB_note2", SNAP_SHOOT, "note2_n5", "n5_return", SNAP_SHOOT));
    autoChooser.addOption(
        "3_startB_n2_n4",
        ezAuto.apply(
            "startB", SHOOT, "startB_note2", SNAP_SHOOT, "note2_n4", "n4_return", SNAP_SHOOT));
    autoChooser.addOption(
        "3_startB_n2_n3",
        ezAuto.apply("startB", SHOOT, "startB_note2", SNAP_SHOOT, "note2_n3", SNAP_SHOOT));
    autoChooser.addOption(
        "3_startB_n2_n7",
        ezAuto.apply(
            "startB",
            SHOOT,
            "startB_note2",
            SNAP_SHOOT,
            "note2_n7",
            "n7_return_under",
            SNAP_SHOOT));
    autoChooser.addOption(
        "3_startC_n3_n7",
        ezAuto.apply(
            "startC",
            SNAP_SHOOT,
            "startC_note3",
            SNAP_SHOOT,
            "note3_n7",
            "n7_return_under",
            SNAP_SHOOT));
    autoChooser.addOption(
        "3_startC_n3_n8",
        ezAuto.apply(
            "startC", SNAP_SHOOT, "startC_note3", SNAP_SHOOT, "note3_n8", "n8_return", SNAP_SHOOT));
    autoChooser.addOption(
        "3_startC_n3_n7",
        ezAuto.apply(
            "startC",
            SNAP_SHOOT,
            "startC_note3",
            SNAP_SHOOT,
            "note3_n7",
            "n7_return_under",
            SNAP_SHOOT));

    autoChooser.addOption(
        "startA_midline_first",
        ezAuto.apply(
            "startA",
            SNAP_SHOOT,
            "startA_n4",
            "n4_return",
            SNAP_SHOOT,
            "note1_n5",
            "n5_return",
            SNAP_SHOOT));
    autoChooser.addOption(
        "startC_midline_first_n7return_out",
        ezAuto.apply(
            "startC",
            SNAP_SHOOT,
            "startC_n8",
            "n8_return",
            SNAP_SHOOT,
            "note3_n7_out",
            "n7_return_out",
            SNAP_SHOOT));
    autoChooser.addOption(
        "startC_midline_first_n7return_under",
        ezAuto.apply(
            "startC",
            SNAP_SHOOT,
            "startC_n8",
            "n8_return",
            SNAP_SHOOT,
            "note3_n7_out",
            "n7_return_under",
            SNAP_SHOOT));

    autoChooser.addOption(
        "4_startB_n1_n2_n3",
        ezAuto.apply(
            "startB",
            SHOOT,
            "startB_note1",
            SNAP_SHOOT,
            "note1_n2",
            SNAP_SHOOT,
            "note2_n3",
            SNAP_SHOOT));

    autoChooser.addOption(
        "4_startB_n3_n2_n1",
        ezAuto.apply(
            "startB",
            SHOOT,
            "startB_note3",
            SNAP_SHOOT,
            "note3_n2",
            SNAP_SHOOT,
            "note2_n1",
            SNAP_SHOOT));

    autoChooser.addOption(
        "4_startA_n1_n2_n6",
        ezAuto.apply(
            "startA",
            SNAP_SHOOT,
            "startA_note1",
            SNAP_SHOOT,
            "note1_n2",
            SNAP_SHOOT,
            "note2_n6",
            "n6_return_under",
            SNAP_SHOOT));
    autoChooser.addOption(
        "4_startA_n1_n2_n3",
        ezAuto.apply(
            "startA",
            SNAP_SHOOT,
            "startA_note1",
            SNAP_SHOOT,
            "note1_n2",
            SNAP_SHOOT,
            "note2_n3",
            SNAP_SHOOT));
    autoChooser.addOption(
        "3_startB_n2_n6",
        ezAuto.apply(
            "startB",
            SHOOT,
            "startB_note2",
            SNAP_SHOOT,
            "note2_n6",
            "n6_return_under",
            SNAP_SHOOT));
    autoChooser.addOption(
        "4_startB_n3_n2_n6",
        ezAuto.apply(
            "startB",
            SHOOT,
            "startB_note3",
            SNAP_SHOOT,
            "note3_n2",
            SNAP_SHOOT,
            "note2_n6",
            "n6_return_under",
            SNAP_SHOOT));
    autoChooser.addOption(
        "4_startC_n3_n2_n1",
        ezAuto.apply(
            "startC",
            SNAP_SHOOT,
            "startC_note3",
            SNAP_SHOOT,
            "note3_n2",
            SNAP_SHOOT,
            "note2_n1",
            SNAP_SHOOT));
    autoChooser.addOption(
        "4_startC_n3_n2_n6",
        ezAuto.apply(
            "startC",
            SNAP_SHOOT,
            "startC_note3",
            SNAP_SHOOT,
            "note3_n2",
            SNAP_SHOOT,
            "note2_n6",
            "n6_return_under",
            SNAP_SHOOT));
    autoChooser.addOption(
        "5_startC_n3_n2_n1_n4",
        ezAuto.apply(
            "startC",
            SNAP_SHOOT,
            "startC_note3",
            SNAP_SHOOT,
            "note3_n2",
            SNAP_SHOOT,
            "note2_n1",
            SNAP_SHOOT,
            "note1_n4",
            "n4_return",
            SNAP_SHOOT));
    /*
     * Drop N Dash auto
     * startD -- n7 -- short/shoot -- n8 -- short/shoot
     */
    autoChooser.addOption(
        "3_startD_n7short_n8short_droppedD",
        ezAuto.apply(
            "startD",
            "startD_n7",
            "n7_return_short",
            SNAP_SHOOT,
            "short_n8",
            "n8_return_short",
            SNAP_SHOOT,
            "short_droppedD",
            SNAP_SHOOT));

    autoChooser.addOption(
        "3_startD_midline_n8_n7_n6",
        ezAuto.apply(
            "startD",
            "startD_n8",
            "n8_return_under",
            SNAP_SHOOT,
            "axis_n7",
            "n7_return_under",
            SNAP_SHOOT,
            "axis_n6",
            SNAP_SHOOT,
            "n6_return_under",
            SNAP_SHOOT));

    autoChooser.addOption(
        "3_startA_dropped_n4_n5",
        ezAuto.apply(
            "startA",
            "startA_n4_out",
            "n4_return",
            SNAP_SHOOT,
            "stage_n5",
            "n5_return",
            SNAP_SHOOT,
            "amp_side_dropped_return",
            SNAP_SHOOT));
    /*
     * Drop N Dash auto
     * startD -- n8 -- short/shoot -- n7 -- short/shoot
     */
    autoChooser.addOption(
        "3_startD_n8short_n7short_droppedD",
        ezAuto.apply(
            "startD",
            "startD_n8",
            "n8_return_short",
            SNAP_SHOOT,
            "short_n7",
            "n7_return_short",
            SNAP_SHOOT,
            "short_droppedD",
            SNAP_SHOOT));

    autoChooser.addOption(
        "3_piece_midline_n4_n5_n6",
        ezAuto.apply(
            "startA",
            "startA_n4_out",
            "n4_return",
            SNAP_SHOOT,
            "stage_n5",
            "n5_return",
            SNAP_SHOOT,
            "stage_n6",
            "n6_return_under",
            SNAP_SHOOT));

    autoChooser.addOption(
        "5_startA_n1_n2_n3_n6",
        ezAuto.apply(
            "startA",
            SNAP_SHOOT,
            "startA_note1",
            SNAP_SHOOT,
            "note1_n2",
            SNAP_SHOOT,
            "note2_n3",
            SNAP_SHOOT,
            "note3_n6",
            "n6_return_under",
            SNAP_SHOOT));

    // autoChooser.addOption(
    //     "4_startA_n1",
    //     ezAuto.apply(
    //         "startA",
    //         SNAP_SHOOT,
    //         "startA_note1",
    //         SNAP_SHOOT,
    //         "note1_n4",
    //         "n4_return",
    //         SNAP_SHOOT,
    //         "note2_n5",
    //         "n5_return",
    //         SNAP_SHOOT));
    // autoChooser.addOption(
    //     "4_startB_n2",
    //     ezAuto.apply(
    //         "startB",
    //         SNAP_SHOOT,
    //         "startB_note2",
    //         SNAP_SHOOT,
    //         "note2_n5",
    //         "n5_return",
    //         SNAP_SHOOT,
    //         "note2_n6",
    //         "n6_return",
    //         SNAP_SHOOT));

    // dispose of the paths, unused paths with be garbage collected.
    paths.dispose();
    return autoChooser;
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

    PPHolonomicDriveController.setRotationTargetOverride(AutoRoutines::turnToNoteOverride);
    AutoCommandBuilder builder = new AutoCommandBuilder(pathPlans, drive, superstructure);
    boolean firstShot = true;
    for (String path : instructions) {
      switch (path) {
        case SHOOT:
        case SNAP_SHOOT:
          if (firstShot) {
            builder.shoot(Constants.ShooterConstants.SETPOINT);
          } else {
            builder.shoot();
          }
          firstShot = false;
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

  private static final Optional<Rotation2d> turnToNoteOverride() {
    if (!isMidlineAuto(AutoCommandBuilder.getName())) {
      return Optional.empty();
    }
    double rot = -PhotonNoteRunnable.getYaw();
    System.out.println("Turning to note (yaw): " + rot);
    if (Math.abs(rot) == 0) {
      return Optional.empty();
    }
    // return Optional.of(
    //     Rotation2d.fromDegrees(
    //         rot + RobotContainer.poseEstimator.getLatestPose().getRotation().getDegrees()));
    return Optional.of(
        RobotContainer.poseEstimator
            .getLatestPose()
            .getRotation()
            .plus(Rotation2d.fromDegrees(rot)));
  }

  /**
   * Checks if a given path is a midline path (defined as ending at the midline) This works with our
   * current naming convnetion (having the target note as the last number)
   *
   * @param pathName - String name of a path
   * @return true if it's a path that ends at the midline, otherwise false
   */
  static boolean isMidlineAuto(String pathName) {
    if (pathName.contains("return")) {
      return false;
    }
    for (int i = pathName.length() - 1; i > 1; i--) {
      char ch = pathName.charAt(i);
      if (Character.isDigit(ch)) {
        int noteNumber = ch - '0';
        return noteNumber >= 4;
      }
    }
    return false;
  }
}
