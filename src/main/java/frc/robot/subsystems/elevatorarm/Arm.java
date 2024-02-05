package frc.robot.subsystems.elevatorarm;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
  private ArmIOTalonFX arm = new ArmIOTalonFX();

  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

  /** Creates a new Flywheel. */
  public Arm() {}

  @Override
  public void periodic() {
    arm.updateTunableNumbers();
    arm.updateInputs(inputs);
    Logger.processInputs("Arm", inputs);
  }

  /** Run open loop at the specified voltage. */
  public void motionmagic1() {
    arm.setmotionmagic();
  }

  public void motionmagic2() {
    arm.setmotionmagic2();
  }

  public String print() {
    return arm.print();
  }

  public void resetarm() {
    arm.resetArm();
  }

  /** Run closed loop at the specified velocity. */
  public void runVelocity(double velocityRPM) {}

  /** Stops the flywheel. */
  public void stop() {
    arm.stop();
  }

  /** Returns the current velocity in RPM. */
  @AutoLogOutput
  public double getVelocityRPM() {
    return 0.0;
  }

  /** Returns the current velocity in radians per second. */
  public double getCharacterizationVelocity() {
    return 0.0;
  }
}
