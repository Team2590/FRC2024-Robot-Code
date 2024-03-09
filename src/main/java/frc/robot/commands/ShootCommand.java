package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Superstructure;
import frc.robot.util.Tracer;

/**
 * Command for shooting using the Superstructure.
 *
 * <p>The command waits for either the superstructure to exit the shooting state or if a certain
 * time has elapsed.
 */
public class ShootCommand extends Command {

  private static final double DEFAULT_SECONDS_TO_WAIT = 0.5;

  private final Timer timer = new Timer();
  private final double timeToWait;
  private final Superstructure superstructure;

  public ShootCommand(Superstructure superstructure, double timeToWait) {
    this.superstructure = superstructure;
    this.timeToWait = timeToWait;
    addRequirements(superstructure.getShooter());
  }

  public ShootCommand(Superstructure superstructure) {
    this(superstructure, DEFAULT_SECONDS_TO_WAIT);
  }

  @Override
  public void initialize() {
    timer.restart();
  }

  @Override
  public void execute() {
    Tracer.trace("ShootCommand.execute(), note_present:" + superstructure.note_present());
    superstructure.shoot();
  }

  @Override
  public boolean isFinished() {
    boolean notePresent = superstructure.note_present();
    Tracer.trace("ShootCommand.isFinished(), notePresent:" + notePresent);
    // If the note is not present anymore, we already shot or don't have the note anymore.
    // This means we exit out of this command.
    return timer.hasElapsed(timeToWait); // || !notePresent;
  }

  @Override
  public void end(boolean interrupted) {
    timer.stop();
    Tracer.trace("ShootCommand.end(), interrupted:" + interrupted);
  }
}
