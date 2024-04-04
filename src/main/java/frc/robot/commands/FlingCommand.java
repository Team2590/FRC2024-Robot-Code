package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Superstructure;
// import frc.robot.util.Tracer;
import org.littletonrobotics.junction.Logger;

/**
 * Command for shooting using the Superstructure.
 *
 * <p>The command waits for either the superstructure to exit the shooting state or if a certain
 * time has elapsed.
 */
public class FlingCommand extends Command {

  private final Timer timer = new Timer();
  private final double timeToWait;
  private final Superstructure superstructure;
  private double shooterPoint;
  private boolean startedShooting = false;
  private long shootStartTime = 0;

  public FlingCommand(Superstructure superstructure, double timeToWait) {
    
  }

  public FlingCommand(Superstructure superstructure, double timeToWait, ) {
    this.superstructure = superstructure;
    this.timeToWait = timeToWait;
    addRequirements(superstructure.getShooter());
  
  }

  @Override
  public void initialize() {
    startedShooting = false;
    timer.restart();
  }

  @Override
  public void execute() {
    if (superstructure.note_present()) { // we can maybe switch this notepresent to a tuned time ?
      superstructure.shoot(shooterPoint);
      if (!startedShooting) {
        startedShooting = true;
        shootStartTime = Logger.getTimestamp();
      }
    }
  }

  @Override
  public boolean isFinished() {
    boolean notePresent = superstructure.note_present();
    return timer.hasElapsed(timeToWait) || (startedShooting && !notePresent);
  }

  @Override
  public void end(boolean interrupted) {
    double shootEndTime = (Logger.getTimestamp() - shootStartTime) / 1000;
    Logger.recordOutput("ShootCommand/ShootTimeMs", shootEndTime);
    timer.stop();
  }
}
