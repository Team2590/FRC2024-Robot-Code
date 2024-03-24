package frc.robot;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.FieldConstants.Targets;
import frc.robot.Superstructure.SuperstructureStates;
import frc.robot.autos.AutoRoutines;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.SnapToTargetCommandTeleop;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.ClimbIOTalonFX;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.conveyor.ConveyorIO;
import frc.robot.subsystems.conveyor.ConveyorIOSim;
import frc.robot.subsystems.conveyor.ConveyorIOTalonFX;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.elevatorarm.Arm;
import frc.robot.subsystems.elevatorarm.ArmIOTalonFX;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.flywheel.FlywheelIO;
import frc.robot.subsystems.flywheel.FlywheelIOSim;
import frc.robot.subsystems.flywheel.FlywheelIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIOTalonFX;
import frc.robot.subsystems.nemesisLED.NemesisLED;
import frc.robot.subsystems.user_input.UserInput;
import frc.robot.subsystems.vision.PhotonNoteRunnable;
import frc.robot.util.PoseEstimator;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private static Drive drive;
  private final Flywheel flywheel;
  private final Conveyor conveyor;
  private final Arm arm;
  private final Intake intake;
  private final Climb climb;
  private final NemesisLED led = new NemesisLED(9, 100);
  private final Superstructure superstructure;
  private final UserInput input;
  public static final PoseEstimator poseEstimator =
      new PoseEstimator(VecBuilder.fill(0.003, 0.003, 0.0002));
  // Dashboard inputs
  public static SnapToTargetCommandTeleop snapCommand;
  private final LoggedDashboardChooser<Command> autoChooser;
  private final PhotonNoteRunnable noteDetection = new PhotonNoteRunnable();
  private final Notifier noteNotifier = new Notifier(noteDetection);
  private boolean teleopSpeaker;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    input = UserInput.getInstance();
    switch (Constants.currentMode) {
      case REAL:
        drive =
            new Drive(
                new GyroIOPigeon2(true),
                new ModuleIOTalonFX(0),
                new ModuleIOTalonFX(1),
                new ModuleIOTalonFX(2),
                new ModuleIOTalonFX(3));
        flywheel = new Flywheel(new FlywheelIOTalonFX());
        // instantiate other subsystems
        conveyor = new Conveyor(new ConveyorIOTalonFX());
        intake = new Intake(new IntakeIOTalonFX());
        arm = new Arm(new ArmIOTalonFX());
        climb = new Climb(new ClimbIOTalonFX());
        noteNotifier.setName("PhotonNoteRunnable");
        noteNotifier.startPeriodic(0.02);
        break;
      case KANG:
        drive =
            new Drive(
                new GyroIOPigeon2(true),
                new ModuleIOTalonFX(0),
                new ModuleIOTalonFX(1),
                new ModuleIOTalonFX(2),
                new ModuleIOTalonFX(3));
        flywheel = new Flywheel(new FlywheelIOTalonFX());
        // instantiate other subsystems
        conveyor = new Conveyor(new ConveyorIOTalonFX());
        intake = new Intake(new IntakeIOTalonFX());
        arm = new Arm(new ArmIOTalonFX());
        climb = new Climb(new ClimbIOTalonFX());
        noteNotifier.setName("PhotonNoteRunnable");
        noteNotifier.startPeriodic(0.02);
        break;
      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
        flywheel = new Flywheel(new FlywheelIOSim());
        // instantiate SIM VERSIONS of other subsystems
        conveyor = new Conveyor(new ConveyorIOSim());
        intake = new Intake(new IntakeIOTalonFX());
        arm = new Arm(new ArmIOTalonFX());
        climb = new Climb(new ClimbIOTalonFX());
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        flywheel = new Flywheel(new FlywheelIO() {});
        conveyor = new Conveyor(new ConveyorIO() {});
        intake = new Intake(new IntakeIOTalonFX());
        climb = new Climb(new ClimbIOTalonFX());
        arm = new Arm(new ArmIOTalonFX());
        break;
    }
    snapCommand =
        new SnapToTargetCommandTeleop(
            drive,
            () -> -input.leftJoystickY(),
            () -> -input.leftJoystickX(),
            Targets.SPEAKER,
            0.05);
    // pass in all subsystems into superstructure
    superstructure = new Superstructure(conveyor, intake, flywheel, arm, climb, led);
    // Set up auto routines
    autoChooser = AutoRoutines.buildChooser(drive, superstructure);
    populateAutoChooser();
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -input.leftJoystickY(),
            () -> -input.leftJoystickX(),
            () -> -input.rightJoystickX()));

    teleopSpeaker = true;
  }

  public void stop() {
    drive.stop();
    superstructure.stop();
  }

  public void updateSubsystems() {
    // Update superstructure since it's not a subsystem.
    // superstructure.periodic();
    input.update();
  }

  public void updateUserInput() {
    Logger.recordOutput("Odometry/Gyro", drive.getGyroYaw().getDegrees());

    /*
     * Driver input w/ superstructure
     */
    // Logger.recordOutput(
    //     "Climb/GetStageAprilTag", poseEstimator.getPhotonRunnable().getStageAprilTag());

    // Logger.recordOutput(
    //     "Climb/HorizontalOffset",
    // poseEstimator.getPhotonRunnable().getHorizontalOffsetToStage());

    // if (input.leftJoystickButton(4)) {
    //   // CommandScheduler.getInstance()
    //   //     .schedule(
    //   //         DriveCommands.alignClimb(
    //   //                 drive,
    //   //                 () -> -input.leftJoystickY(),
    //   //                 poseEstimator.getPhotonRunnable().getStageAprilTag(),
    //   //                 poseEstimator.getPhotonRunnable().getHorizontalOffsetToStage())
    //   //             .until(() -> input.leftJoystickButton(4)));
    //   superstructure.climb();
    //   // superstructure.primeShooter();
    // }

    if (input.controllerAButton()) {
      superstructure.primeShooter();
    } else if (input.controllerBButton()) {
      superstructure.stopShooter();
    }

    if (input.controllerXButton()) {
      superstructure.runConveyor();
    } else if (input.controllerYButton()) {
      superstructure.stopConveyor();
    }

    if (input.leftJoystickTrigger()) {
      if (teleopSpeaker) {
        CommandScheduler.getInstance()
            .schedule(
                DriveCommands.SnapToTarget(
                        drive,
                        () -> -input.leftJoystickY(),
                        () -> -input.leftJoystickX(),
                        Targets.SPEAKER)
                    .until(() -> input.leftJoystickTrigger()));
        superstructure.shoot();
      } else {
        superstructure.scoreAmp();
      }
    } else if (input.rightJoystickTrigger()) {
      superstructure.intake();
    } else if (PhotonNoteRunnable.target != null && input.rightJoystickButton(2)) {
      // I just put this button as a place holder
      CommandScheduler.getInstance()
          .schedule(
              DriveCommands.turnToNote(
                      drive,
                      () -> -input.leftJoystickY(),
                      () -> -input.leftJoystickX(),
                      PhotonNoteRunnable.target::getYaw)
                  .until(() -> input.rightJoystickButton(2)));
    } else if (input.rightJoystickButton(11)) {
      // manual arm w climb DOESNT WORK
      superstructure.flip();
      // superstructure.climb();
    } else if (input.rightJoystickButton(16)) {
      superstructure.climb();
    } else if (input.rightJoystickButton(5)) {
      System.out.println("zeroing gyro");
      drive.zeroGyro();
    } else if (input.leftJoystickPOV() == 180) {
      superstructure.subwooferShot();
    } else if (input.rightJoystickPOV() == 180) {
      // spit
      CommandScheduler.getInstance()
          .schedule(
              DriveCommands.alignClimb(
                      drive,
                      () -> -input.leftJoystickY(),
                      poseEstimator.getPhotonRunnable().getStageAprilTag(),
                      poseEstimator.getPhotonRunnable().getHorizontalOffsetToStage())
                  .until(() -> input.rightJoystickPOV() == 180));
    } else if (input.rightJoystickPOV() == 90) {
      superstructure.outtake();
    } else if (input.rightJoystickButton(3)) {
      // highkey does not work rn
      teleopSpeaker = false;
      input.setOperatorRumble(0);
      superstructure.primeAmp();
    } else if (input.rightJoystickButton(4)) {
      superstructure.flip();
    } else if (input.leftJoystickButton(4)) {
      superstructure.climb();
    }
    // else if (input.leftJoystickButton(4)) {
    //   superstructure.climb();
    // }
    else if (input.controllerButton(7)) {
      superstructure.resetRobot();
    } else if (input.rightJoystickButton(15)) {
      DriveCommands.SnapToTarget(
              drive, () -> -input.leftJoystickY(), () -> -input.leftJoystickX(), Targets.FLING)
          .until(() -> input.rightJoystickButton(2));
      superstructure.fling();
    } else {

      if (superstructure.getState() == SuperstructureStates.PRIMING_AMP
          || superstructure.getState() == SuperstructureStates.IDLE_AMP) {
        teleopSpeaker = false;
      } else {
        teleopSpeaker = true;
      }

      superstructure.idle();
    }

    // Logger.recordOutput("shoot speaker?", teleopSpeaker);
    // if (input.controllerXButton()) {
    //   superstructure.armClimb();
    // }
    // TBD OPERATOR BUTTONS

    // if (input.leftJoystickTrigger()) {
    //   superstructure.intake();
    // } else if (input.rightJoystickTrigger()) {
    //   superstructure.outtake();
    // } else if (input.rightJoystickButton(2)) {
    //   CommandScheduler.getInstance()
    //       .schedule(
    //           DriveCommands.SnapToTarget(
    //                   drive,
    //                   () -> -input.leftJoystickY(),
    //                   () -> -input.leftJoystickX(),
    //                   Targets.SPEAKER)
    //               .until(() -> input.rightJoystickButton(2)));
    //   // Example Use below
    //   // CommandScheduler.getInstance()
    //   //     .schedule(
    //   //         DriveCommands.turnToNote(
    //   //                 drive,
    //   //                 () -> -input.leftJoystickY(),
    //   //                 () -> -input.leftJoystickX(),
    //   //                 PhotonNoteRunnable.target::getYaw)
    //   //             .until(() -> input.rightJoystickButton(2)));
    //   superstructure.shoot();
    // } else if (input.rightJoystickButton(3)) {
    //   superstructure.scoreAmp();
    // } else if (input.rightJoystickButton(5)) {
    //   drive.zeroGyro();
    //   System.out.println("Gyro is reset");
    // } else if (input.leftJoystickButton(2)) {
    //   superstructure.armUp();
    // } else if (input.leftJoystickButton(3)) {
    //   superstructure.armDown();
    // } else if (input.rightJoystickButton(6)) {
    //   superstructure.climb();
    // } else {
    //   superstructure.idle();
    // }
  }

  // --------AUTO CHOOSER FUNCTIONS------------
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // superstructure
    //     .getShooter()
    //     .setDefaultCommand(
    //         new InstantCommand(() -> superstructure.primeShooter(),
    // superstructure.getShooter()));
    return autoChooser.get();
  }

  public static Drive getDrive() {
    return drive;
  }

  // public void initGyro() {

  //   double
  // gyroreset=DriverStation.getAlliance().isPresent()&&DriverStation.getAlliance().get()==Alliance.Red?180:0;
  //   drive.set

  // }

  /**
   * Use this branch to build the commands from the autos that you want to run, and add the commands
   * to the AutoChooser.
   */
  private void populateAutoChooser() {
    // Set up feedforward characterization
    // autoChooser.addOption(
    //     "Drive FF Characterization",
    //     new FeedForwardCharacterization(
    //         drive, drive::runCharacterizationVolts, drive::getCharacterizationVelocity));
    // autoChooser.addOption(
    //     "Flywheel FF Characterization",
    //     new FeedForwardCharacterization(
    //         flywheel, flywheel::runVolts, flywheel::getCharacterizationVelocity));
  }
}
