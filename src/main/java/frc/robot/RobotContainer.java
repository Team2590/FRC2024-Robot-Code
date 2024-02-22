package frc.robot;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.autos.AutoRoutines;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.FeedForwardCharacterization;
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
import frc.robot.subsystems.user_input.UserInput;
import frc.robot.util.PoseEstimator;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Flywheel flywheel;
  private final Conveyor conveyor;
  private final Arm arm;
  private final Intake intake;
  private final Climb climb;
  private final Superstructure superstructure;
  private final UserInput input;
  public static final PoseEstimator poseEstimator =
      new PoseEstimator(VecBuilder.fill(0.003, 0.003, 0.0002));
  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  private Command snapDrive;

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
        arm = new Arm(new ArmIOTalonFX());
        climb = new Climb(new ClimbIOTalonFX());
        break;
    }
    // pass in all subsystems into superstructure
    superstructure = new Superstructure(conveyor, intake, flywheel, arm, climb);
    // Set up auto routines
    autoChooser = AutoRoutines.buildChooser(drive, superstructure);
    populateAutoChooser();
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -input.leftJoystickY(),
            () -> -input.leftJoystickX(),
            () -> input.rightJoystickX()));
  }

  public void stop() {
    drive.stop();
    superstructure.stop();
  }

  public void updateSubsystems() {
    // call update functions of all subsystems
    superstructure.periodic();
    // conveyor.periodic();
    // flywheel.periodic();
    // intake.periodic();
    // arm.periodic();
    input.update();
  }

  public void updateUserInput() {
    // joystick inputs galore!
    // if (input.leftJoystickTriggerPressed()) {
    //   snapDrive =
    //       DriveCommands.SnapToTarget(
    //           drive, () -> input.leftJoystickY(), () -> input.leftJoystickX(), new Pose2d());
    //   CommandScheduler.getInstance().schedule(snapDrive);
    // } else if (input.leftJoystickButtonReleased(1)) {
    //   CommandScheduler.getInstance().cancel(snapDrive);
    // }

    /*
     * Driver input w/ superstructure
     */
    if (input.leftJoystickTrigger()) {
      superstructure.intake();
    } else if (input.rightJoystickTrigger()) {
      superstructure.outtake();
    } else if (input.rightJoystickButton(2)) {
      superstructure.shoot();
    } else if (input.rightJoystickButton(3)) {
      superstructure.scoreAmp();
    } else if (input.rightJoystickButton(5)) {
      drive.zeroGyro();
      System.out.println("Gyro is reset");
    } else if (input.leftJoystickButton(2)) {
      superstructure.armUp();
    } else if (input.leftJoystickButton(3)) {
      superstructure.armDown();
    } else if (input.leftJoystickButton(5)) {
      superstructure.climb();
    } else {
      superstructure.idle();
    }
  }

  // --------AUTO CHOOSER FUNCTIONS------------
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
  /**
   * Use this branch to build the commands from the autos that you want to run, and add the commands
   * to the AutoChooser.
   */
  private void populateAutoChooser() {

    // Set up feedforward characterization
    autoChooser.addOption(
        "Drive FF Characterization",
        new FeedForwardCharacterization(
            drive, drive::runCharacterizationVolts, drive::getCharacterizationVelocity));
    // autoChooser.addOption(
    //     "Flywheel FF Characterization",
    //     new FeedForwardCharacterization(
    //         flywheel, flywheel::runVolts, flywheel::getCharacterizationVelocity));
  }

  /**
   * Use this to register all of the commands with the AutoBuilder This should include all commands
   * used in the autos (except drive commands)
   */
  private void registerAutoCommands() {
    // NamedCommands.registerCommand(
    //     "Run Flywheel",
    //     Commands.startEnd(
    //             () -> flywheel.runVelocity(flywheelSpeedInput.get()),
    //             flywheel::setStopped,
    //             flywheel)
    //         .withTimeout(5.0));
  }
  // private void populateAutoChooser() {
  //   // Set up feedforward characterization
  //   autoChooser.addOption(
  //       "Drive FF Characterization",
  //       new FeedForwardCharacterization(
  //           drive, drive::runCharacterizationVolts, drive::getCharacterizationVelocity));
  //   autoChooser.addOption(
  //       "Flywheel FF Characterization",
  //       new FeedForwardCharacterization(
  //           flywheel, flywheel::runVolts, flywheel::getCharacterizationVelocity));
  // }
}
