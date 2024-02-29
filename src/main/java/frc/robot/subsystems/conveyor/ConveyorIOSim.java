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
  private final DCMotorSim feederSim = new DCMotorSim(DCMotor.getFalcon500(1), ConveyorConstants.FEEDER_GEAR_RATIO, 1.0);
  private final DCMotorSim diverterSim = new DCMotorSim(DCMotor.getFalcon500(1), ConveyorConstants.DIVERTER_GEAR_RATIO, 1.0);
  private final AnalogInputSim intakeProxSim = new AnalogInputSim(ConveyorConstants.INTAKE_PROX_ID);
  private final AnalogInputSim shooterProxSim =
      new AnalogInputSim(ConveyorConstants.SHOOTER_PROX_ID);
  private double feederAppliedVolts;
  private double diverterAppliedVolts;

  @Override
  public void updateInputs(ConveyorIOInputs inputs) {
    inputs.detectedIntakeSide = detectedIntakeSide();
    inputs.intakeProxVolts = intakeProxSim.getVoltage();
    inputs.detectedShooterSide = detectedShooterSide();
    inputs.shooterProxVolts = shooterProxSim.getVoltage();
    inputs.feederRPM = feederSim.getAngularVelocityRPM();
    inputs.feederVolts = feederAppliedVolts;
    inputs.diverterRPM = diverterSim.getAngularVelocityRPM();
    inputs.diverterVolts = diverterAppliedVolts;
    inputs.currentAmps = new double[] {Math.abs(feederSim.getCurrentDrawAmps()),Math.abs(diverterSim.getCurrentDrawAmps())};
    inputs.hasNote = noteInConveyor();
  }

  @Override
  public void runPower(double power) {
    feederAppliedVolts = 10 * power;
    diverterAppliedVolts = 10 * power;
    feederSim.setInputVoltage(feederAppliedVolts);
    diverterSim.setInputVoltage(diverterAppliedVolts);
  }

  @Override
  public void runPower(double feederPower, double diverterPower) {
    feederAppliedVolts = 10 * feederPower;
    diverterAppliedVolts = 10 * diverterPower;
    feederSim.setInputVoltage(feederAppliedVolts);
    diverterSim.setInputVoltage(diverterAppliedVolts);
  }

  /**
   * Checks if there is something detected by the intake side prox sensor
   *
   * @return if something is detected
   */
  private boolean detectedIntakeSide() {
    return intakeProxSim.getVoltage() > ConveyorConstants.INTAKE_PROX_THRESHOLD;
  }

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
    return detectedIntakeSide() && detectedShooterSide();
  }
}
