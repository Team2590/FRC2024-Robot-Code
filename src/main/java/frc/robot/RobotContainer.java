package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.autos.AutoRoutines;
import frc.robot.commands.DriveCommands;
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
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.flywheel.FlywheelIO;
import frc.robot.subsystems.flywheel.FlywheelIOSim;
import frc.robot.subsystems.flywheel.FlywheelIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIOTalonFX;
import frc.robot.subsystems.user_input.UserInput;
import frc.util.PoseEstimator;

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
  private final Intake intake;
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
                new GyroIOPigeon2(true) {},
                new ModuleIOTalonFX(0),
                new ModuleIOTalonFX(1),
                new ModuleIOTalonFX(2),
                new ModuleIOTalonFX(3));
        flywheel = new Flywheel(new FlywheelIOTalonFX());
        // instantiate other subsystems
        conveyor = new Conveyor(new ConveyorIOTalonFX());
        intake = new Intake(new IntakeIOTalonFX());
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
        // instantiate SIM VERSIONS F other subsystems
        conveyor = new Conveyor(new ConveyorIOSim());
        intake = new Intake(new IntakeIOTalonFX());
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
        break;
    }
    // pass in all subsystems into superstructure
    superstructure = new Superstructure(conveyor, intake);
    // Set up auto routines
    autoChooser = AutoRoutines.buildChooser(drive, superstructure);
    // registerAutoCommands();
    // populateAutoChooser();
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
    conveyor.periodic();
    input.update();
  }

  public void updateUserInput() {
    // joystick inputs galore!
    if (input.leftJoystickTriggerPressed()) {
      snapDrive =
          DriveCommands.SnapToTarget(
              drive, () -> input.leftJoystickY(), () -> input.leftJoystickX(), new Pose2d());
      CommandScheduler.getInstance().schedule(snapDrive);
    } else if (input.leftJoystickButtonReleased(1)) {
      CommandScheduler.getInstance().cancel(snapDrive);
    }
    if (input.rightJoystickTriggerPressed()) {
      superstructure.intake();
    }
    // joystick inputs galore!
    // if (input.leftJoystickTrigger()) {
    // superstructure.intake();
    // superstructure.intake();
    // } else if (input.leftJoystickButton(2)) {
    // superstructure.amp();
    // } else {
    // superstructure.stop();
    // }
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
