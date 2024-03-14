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
  private final double timeToWait;
  private final Superstructure superstructure;
  private boolean isNoteDetectedAtIntake = false;

  public ShootCommand(Superstructure superstructure, double timeToWait) {
    this.superstructure = superstructure;
    this.timeToWait = timeToWait;
    addRequirements(superstructure.getShooter());
  }

  @Override
  public void initialize() {
    timer.restart();
    isNoteDetectedAtIntake = superstructure.getIntake().detectNoteForAuton();
    System.out.println("---- initialize:" + isNoteDetectedAtIntake);
  }

  @Override
  public void execute() {
    superstructure.shoot();
  }

  @Override
  public boolean isFinished() {
    boolean notePresent = superstructure.note_present();
    if (isNoteDetectedAtIntake && !notePresent) {
      // The intake detected the note but it hasn't made it's way to the conveyor
      // so we wait
      System.out.println(
          "--- waiting for note, notePresent:"
              + notePresent
              + " , isNoteDetectedAtIntake:"
              + isNoteDetectedAtIntake);
      return false;
    }
    // note not present means the we shot the note.
    // return !notePresent; // || timer.hasElapsed(timeToWait);
    System.out.println("----- ShootCommand:" + superstructure.note_present());
    return !superstructure.note_present();
  }

  @Override
  public void end(boolean interrupted) {
    timer.stop();
    superstructure.getIntake().resetDetectNoteForAuton();
  }
}
