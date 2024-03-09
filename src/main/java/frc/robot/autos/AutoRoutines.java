package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Superstructure;
import frc.robot.subsystems.drive.Drive;
import java.util.function.Function;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class AutoRoutines {

  private static final String SHOOT = "shoot";
  private static final String SNAP_SHOOT = "snap_shoot";
  private static final String INTAKE = "intake";

  public static final LoggedDashboardChooser<Command> buildChooser(
      Drive drive, Superstructure superstructure) {
    LoggedDashboardChooser<Command> autoChooser = new LoggedDashboardChooser<>("Auto Choices");

    PathPlannerPaths paths = PathPlannerPaths.create();
    // An easy way to create autos without having to supply paths, drive and superstructure for
    // every call.
    AutoFunction ezAuto = (instructions) -> buildAuto(paths, drive, superstructure, instructions);

    // Register all the auto routines here

    autoChooser.addOption(
        "2_startA_n1", ezAuto.apply("startA", SNAP_SHOOT, "startA_note1", SNAP_SHOOT));
    autoChooser.addOption(
        "2_startC_n3", ezAuto.apply("startC", SNAP_SHOOT, "startC_note3", SNAP_SHOOT));
    autoChooser.addOption(
        "2_startB_n2", ezAuto.apply("startB", SNAP_SHOOT, "startB_note2", SNAP_SHOOT));
    autoChooser.addOption(
        "3_startA_n1_n4",
        ezAuto.apply(
            "startA", SNAP_SHOOT, "startA_note1", SNAP_SHOOT, "note1_n4", "n4_return", SNAP_SHOOT));
    autoChooser.addOption(
        "3_startB_n2_n1",
        ezAuto.apply("startB", SNAP_SHOOT, "startB_note2", SNAP_SHOOT, "note2_n1", SNAP_SHOOT));
    autoChooser.addOption(
        "3_startB_n2_n5",
        ezAuto.apply(
            "startB", SNAP_SHOOT, "startB_note2", SNAP_SHOOT, "note2_n5", "n5_return", SNAP_SHOOT));
    autoChooser.addOption(
        "3_startB_n2_n4",
        ezAuto.apply(
            "startB", SNAP_SHOOT, "startB_note2", SNAP_SHOOT, "note2_n4", "n4_return", SNAP_SHOOT));
    autoChooser.addOption(
        "3_startB_n2_n3",
        ezAuto.apply("startB", SNAP_SHOOT, "startB_note2", SNAP_SHOOT, "note2_n3", SNAP_SHOOT));
    autoChooser.addOption(
        "3_startB_n2_n7",
        ezAuto.apply(
            "startB",
            SNAP_SHOOT,
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
    // autoChooser.addOption(
    //     "3_startC_n3_n7",
    //     ezAuto.apply(
    //         "startC",
    //         SNAP_SHOOT,
    //         "startC_note3",
    //         SNAP_SHOOT,
    //         "note3_n7",
    //         "n7_return_under",
    //         SNAP_SHOOT));

    autoChooser.addOption(
        "outside_first",
        ezAuto.apply(
            "startC",
            SNAP_SHOOT,
            "n8_outside",
            "n8_return",
            SNAP_SHOOT,
            "note3_n7",
            "n7_return",
            SNAP_SHOOT));

    autoChooser.addOption(
        "4_startB_close",
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
        "4_startA_n1_n2_n3",
        ezAuto.apply(
            "startA",
            SHOOT,
            "startA_note1",
            SNAP_SHOOT,
            "note1_n2",
            SNAP_SHOOT,
            "note2_n3",
            SNAP_SHOOT));

    autoChooser.addOption(
        "4_startc_n3_n2_n1",
        ezAuto.apply(
            "startC",
            SHOOT,
            "startC_note3",
            SNAP_SHOOT,
            "note3_n2",
            SNAP_SHOOT,
            "note2_n1",
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
    // autoChooser.addOption(
    //     "StartA",
    //     ezAuto.apply(
    //         "startA", // Starting Position A
    //         SNAP_SHOOT,
    //         "startA_note1",
    //         SNAP_SHOOT));

    // autoChooser.addOption(
    //     "StartB",
    //     ezAuto.apply(
    //         "startB", // Starting Position B
    //         SNAP_SHOOT,
    //         "startB_note1",
    //         SNAP_SHOOT,
    //         "n2-n3",
    //         SNAP_SHOOT));
            

    // autoChooser.addOption(
    //     "StartC",
    //     ezAuto.apply(
    //         "startC", // runs path startC
    //         SNAP_SHOOT,
    //         "startC_note3",
    //         SNAP_SHOOT));

    // dispose of the paths, unused paths with be garbage collected.
    paths.dispose();
    return autoChooser;
  }

  /**
   * Easy way to configure an Auto Routine, just pass in the paths.
   *
   * <p>IMPORTANT!!!! The first instruction needs to be the path which is used to reset the robot
   * pose otherwise we will crash at start up.
   *
   * <p>Use 'shoot' for just shooting or 'snap_shoot' for snaping to target and shooting or 'intake'
   * for running intake.
   */
  private static Command buildAuto(
      PathPlannerPaths pathPlans,
      Drive drive,
      Superstructure superstructure,
      String... instructions) {
    AutoCommandBuilder builder = new AutoCommandBuilder(pathPlans, drive, superstructure);
    // Since SnapToTarget requires that we have initialized our pose, the first path that gets
    // passed in is used only for resetting the pose.
    String pathToUseForResettingPose = instructions[0];
    builder.resetPoseUsingPath(pathToUseForResettingPose);

    boolean isFirstShoot = true;
    for (int i = 1; i < instructions.length; i++) {
      String instruction = instructions[i];
      switch (instruction) {
        case SHOOT:
          // builder.shoot(false);
          // if (i == 1) {

          // }
          // break;
        case SNAP_SHOOT:
          if (isFirstShoot) {
            // This is the first shoot command
            builder.shoot(true, 1.5d);
            isFirstShoot = false;
          } else {
            builder.shoot(true);
          }

          break;
        case INTAKE:
          // intake automatically starts up after shoot but just in case if we need it.
          builder.intake();
          break;
        default:
          // if this was to be the first path, AutoCommandBuilder will make it the StartPathCommand.
          builder.followPath(instruction);
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
}
