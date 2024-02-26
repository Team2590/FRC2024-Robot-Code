package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Superstructure;

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
    System.out.println("Shooting .... ");
    superstructure.shoot();
  }

  @Override
  public boolean isFinished() {
    return (!superstructure.isShooting()) || (timer.hasElapsed(timeToWait));
    // TODO create a condition to figure out when shooting is done- conveyor, !hasnote
  }

  @Override
  public void end(boolean interrupted) {
    timer.stop();
    // start intake immediately after shooting
    superstructure.intake();
  }
}
