package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climb extends SubsystemBase {
  private ClimbIOTalonFX climbMotors = new ClimbIOTalonFX();
  private ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();

  @Override
  public void periodic() {
    System.out.println(climbMotors.getPosition());
    System.out.println(climbMotors.maxPosition);
    climbMotors.updateInputs(inputs);
  }

  public void up() {
    climbMotors.up();
  }

  public void down() {
    climbMotors.down();
  }

  public void stop() {
    climbMotors.stop();
  }

  public void reset() {
    climbMotors.reset();
  }

  public void resetRotationCount() {
    climbMotors.resetRotationCount();
  }
}
