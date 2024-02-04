package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climb extends SubsystemBase {
  private ClimbIOTalonFX climbMotors = new ClimbIOTalonFX();
  private ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();
  private ClimbStates state;


  @Override
  public void periodic() {
    System.out.println(climbMotors.getPosition());
    System.out.println(climbMotors.maxPosition);
    climbMotors.updateInputs(inputs);
  }

  public void up() {
    climbMotors.up();
    state = ClimbStates.MANUAL;
  }

  public void down() {
    climbMotors.down();
    state = ClimbStates.MANUAL;
  }

  public void stop() {
    climbMotors.stop();
    state = ClimbStates.STOPPED;
  }

  public void resetRotationCount() {
    climbMotors.resetRotationCount();
    state = ClimbStates.MANUAL;
  }

  public ClimbStates getState() {
    return state;
  }
}
