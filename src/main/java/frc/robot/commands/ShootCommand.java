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

  private static final double SECONDS_TO_WAIT = 1;

  private final Timer timer = new Timer();
  private final Superstructure superstructure;

  public ShootCommand(Superstructure superstructure) {
    this.superstructure = superstructure;
    addRequirements(superstructure.getShooter());
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
    return (!superstructure.isShooting()) && (timer.hasElapsed(SECONDS_TO_WAIT));
  }

  @Override
  public void end(boolean interrupted) {
    timer.stop();
    // start intake immediately after shooting
    superstructure.intake();
  }
}
