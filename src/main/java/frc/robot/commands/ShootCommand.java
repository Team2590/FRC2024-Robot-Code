package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Superstructure;
import org.littletonrobotics.junction.Logger;

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
    Logger.recordOutput("Auto/Trace", "Running ShootCommand");
    superstructure.shoot();
  }

  @Override
  public boolean isFinished() {
    // If the shooter side prox sensor of conveyor doesn't detect note, shoot is done.
    boolean detectedShooterSideNote = superstructure.getConveyor().detectedShooterSide();
    Logger.recordOutput("Auto/Trace", "Conveyer ShooterSideHasNote" + detectedShooterSideNote);
    return timer.hasElapsed(timeToWait) || !detectedShooterSideNote;
  }

  @Override
  public void end(boolean interrupted) {
    timer.stop();
    Logger.recordOutput("Auto/Trace", "ShootCommand Done.");
  }
}
