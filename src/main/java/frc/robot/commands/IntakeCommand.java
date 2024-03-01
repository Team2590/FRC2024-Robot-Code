package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Superstructure;
import frc.robot.util.Tracer;

/** Runs the intake. */
public class IntakeCommand extends Command {

  private final Superstructure superstructure;
  private final double waitTimeSeconds;
  private final Timer timer = new Timer();

  public IntakeCommand(Superstructure superstructure, double waitTimeSeconds) {
    this.superstructure = superstructure;
    this.waitTimeSeconds = waitTimeSeconds;
    addRequirements(superstructure.getIntake());
  }

  @Override
  public void initialize() {
    timer.restart();
  }

  @Override
  public void execute() {
    Tracer.trace("Running IntakeCommand");
    superstructure.intake();
  }

  @Override
  public boolean isFinished() {
    return timer.hasElapsed(waitTimeSeconds);
  }

  @Override
  public void end(boolean interrupted) {
    Tracer.trace("IntakeCommand Done.");
  }
}