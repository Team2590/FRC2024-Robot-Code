package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Superstructure;
// import frc.robot.util.Tracer;

/**
 * Command for shooting using the Superstructure.
 *
 * <p>The command waits for either the superstructure to exit the shooting state or if a certain
 * time has elapsed.
 */
public class ShootCommand extends Command {

  private final Timer timer = new Timer();
  private final Timer shooterTimer = new Timer();
  private static final double DEFAULT_SECONDS_TO_WAIT = 3.0;
  private final double timeToWait;
  private final Superstructure superstructure;
  private boolean isNoteDetectedAtIntake = false;
  private int cycle = 0;
  private int shooterPoint;
  private boolean shooting = false;

  public ShootCommand(Superstructure superstructure, double timeToWait) {
    this.superstructure = superstructure;
    this.timeToWait = timeToWait;
    addRequirements(superstructure.getShooter());
    shooterPoint = 2300;
  }

  public ShootCommand(Superstructure superstructure) {
    this(superstructure, DEFAULT_SECONDS_TO_WAIT);
    shooterPoint = 2300;
  }

  public ShootCommand(Superstructure superstructure, double timeToWait, int setpoint) {
    this.superstructure = superstructure;
    this.timeToWait = timeToWait;
    addRequirements(superstructure.getShooter());
    shooterPoint = setpoint;
  }

  @Override
  public void initialize() {
    shooting = false;
    timer.restart();
    shooterTimer.reset();
    // isNoteDetectedAtIntake = superstructure.getIntake().detectNoteForAuton();
    // if(!superstructure.getIntake().detectNote() && superstructure.note_present())
    //   shooterPoint=900;
  }

  @Override
  public void execute() {
    // isNoteDetectedAtIntake =
    //     superstructure.getIntake().detectNoteForAuton() || superstructure.note_present();
    // Tracer.trace("ShootCommand.execute(), Intake.detectNote:" + isNoteDetectedAtIntake);

    /*if note present then shoot ? otherwise freeze the timer or something like that */
    if (superstructure.note_present()) {
      superstructure.shoot(shooterPoint);
      shooterTimer.start();
      timer.stop();
    }

    // if (!superstructure.note_present()) {
    //   cycle++;
    // }
  }

  @Override
  public boolean isFinished() {
    boolean notePresent = superstructure.note_present();
    if (timer.hasElapsed(timeToWait) || shooterTimer.hasElapsed(1)) {
      return true;
    } else {
      return false;
    }
    // return !notePresent || timer.hasElapsed(timeToWait);
  }

  @Override
  public void end(boolean interrupted) {
    timer.stop();
    // Tracer.trace("ShootCommand.end(), interrupted:" + interrupted);
    // superstructure.getIntake().resetDetectedNoteForAuton();
  }
}
