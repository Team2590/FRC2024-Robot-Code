package frc.robot.subsystems.conveyor;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.AnalogInputSim;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.ConveyorConstants;

/**
 * IO class for a simulated conveyor subsystem
 *
 * @author Ian Keller
 */
public class ConveyorIOSim implements ConveyorIO {
  // TODO: find the 2 guessed values (they also may not be that important because the actual
  // movement is not that important)
  private final DCMotorSim feederSim = new DCMotorSim(DCMotor.getFalcon500(1), 1.0, 1.0);
  private final DCMotorSim diverterSim = new DCMotorSim(DCMotor.getFalcon500(1), 1.0, 1.0);
  private final AnalogInputSim shooterProxSim =
      new AnalogInputSim(ConveyorConstants.SHOOTER_PROX_ID);

  @Override
  public void updateInputs(ConveyorIOInputs inputs) {
    inputs.detectedShooterSide = detectedShooterSide();
    inputs.shooterProxVolts = shooterProxSim.getVoltage();
    inputs.hasNote = noteInConveyor();
  }

  @Override
  public void runPower(double power) {
    feederSim.setInput(power);
    diverterSim.setInput(power);
  }

  @Override
  public void runPower(double feederPower, double diverterPower) {
    feederSim.setInput(feederPower);
    diverterSim.setInput(diverterPower);
  }

  /**
   * Checks if there is something detected by the intake side prox sensor
   *
   * @return if something is detected
   */

  /**
   * Checks if there is something detected by the shooter side prox sensor
   *
   * @return if something is detected
   */
  private boolean detectedShooterSide() {
    return shooterProxSim.getVoltage() > ConveyorConstants.SHOOTER_PROX_THRESHOLD;
  }

  /**
   * Uses the two prox sensors to determine if there is a note being stowed
   *
   * @return if there is a note in resting pos
   */
  private boolean noteInConveyor() {
    return detectedShooterSide();
  }
}
