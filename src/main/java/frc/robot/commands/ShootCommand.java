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

  public ShootCommand(Superstructure superstructure, double timeToWait) {
    this.superstructure = superstructure;
    this.timeToWait = timeToWait;
    addRequirements(superstructure.getShooter());
  }

  @Override
  public void initialize() {
    timer.restart();
    // isNoteDetectedAtIntake = superstructure.getIntake().detectNoteForAuton();
  }

  @Override
  public void execute() {
    // isNoteDetectedAtIntake =
    //     superstructure.getIntake().detectNoteForAuton() || superstructure.note_present();
    superstructure.shoot();
  }

  @Override
  public boolean isFinished() {
    boolean notePresent = superstructure.note_present();
    return !notePresent || timer.hasElapsed(timeToWait);
  }

  @Override
  public void end(boolean interrupted) {
    timer.stop();
    // Tracer.trace("ShootCommand.end(), interrupted:" + interrupted);
    // superstructure.getIntake().resetDetectedNoteForAuton();
  }
}
