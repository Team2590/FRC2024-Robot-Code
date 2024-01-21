package frc.robot.subsystems.elevatorarm;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.AutoLogOutput;

public class Arm extends SubsystemBase {
  private final ArmIO io;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
  private final SimpleMotorFeedforward ffModel;

  /** Creates a new Flywheel. */
  public Arm(ArmIO io) {
    this.io = io;

    // Switch constants based on mode (the physics simulator is treated as a
    // separate robot with different tuning)
    switch (Constants.currentMode) {
      case REAL:
      case REPLAY:
        ffModel = new SimpleMotorFeedforward(0.1, 0.05);
        // io.configurePID(1.0, 0.0, 0.0);
        break;
      case SIM:
        ffModel = new SimpleMotorFeedforward(0.0, 0.03);
        // io.configurePID(0.5, 0.0, 0.0);
        break;
      default:
        ffModel = new SimpleMotorFeedforward(0.0, 0.0);
        break;
    }
  }

  @Override
  public void periodic() {}

  /** Run open loop at the specified voltage. */
  public void runVolts(double volts) {}

  /** Run closed loop at the specified velocity. */
  public void runVelocity(double velocityRPM) {}

  /** Stops the flywheel. */
  public void stop() {}

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
