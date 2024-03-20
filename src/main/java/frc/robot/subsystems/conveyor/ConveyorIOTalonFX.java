package frc.robot.subsystems.conveyor;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.AnalogInput;
import frc.robot.Constants;
import frc.robot.Constants.ConveyorConstants;

/**
 * IO class for a real conveyor subsystem
 *
 * @author Ian Keller
 */
public class ConveyorIOTalonFX implements ConveyorIO {
  // motors + sensors
  private final TalonFX feederMotor = new TalonFX(Constants.ConveyorConstants.FEEDER_ID);
  private final TalonFX diverterMotor = new TalonFX(ConveyorConstants.DIVRETER_ID);
  private final AnalogInput shooterProx = new AnalogInput(ConveyorConstants.SHOOTER_PROX_ID);

  private final StatusSignal<Double> feederVelocity = feederMotor.getVelocity();
  private final StatusSignal<Double> feederAppliedVolts = feederMotor.getMotorVoltage();
  private final StatusSignal<Double> feederCurrent = feederMotor.getStatorCurrent();

  private final StatusSignal<Double> diverterVelocity = diverterMotor.getVelocity();
  private final StatusSignal<Double> diverterAppliedVolts = diverterMotor.getMotorVoltage();
  private final StatusSignal<Double> diverterCurrent = diverterMotor.getStatorCurrent();

  public ConveyorIOTalonFX() {
    var config = new TalonFXConfiguration();
    config.CurrentLimits.StatorCurrentLimit = 30.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.MotorOutput.Inverted = ConveyorConstants.feederDirection;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    feederMotor.getConfigurator().apply(config);
    config.MotorOutput.Inverted = ConveyorConstants.diverterDirection;
    diverterMotor.getConfigurator().apply(config);

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        feederVelocity,
        feederAppliedVolts,
        feederCurrent,
        diverterVelocity,
        diverterAppliedVolts,
        diverterCurrent);
    feederMotor.optimizeBusUtilization();
    diverterMotor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ConveyorIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        feederVelocity,
        feederAppliedVolts,
        feederCurrent,
        diverterVelocity,
        diverterAppliedVolts,
        diverterCurrent);
    inputs.detectedShooterSide = detectedShooterSide();
    inputs.shooterProxVolts = shooterProx.getVoltage();
    inputs.hasNote = noteInConveyor();
    inputs.diverterRPM =
        diverterVelocity.getValueAsDouble() * ConveyorConstants.DIVERTER_GEAR_RATIO;
    inputs.feederVolts = feederAppliedVolts.getValueAsDouble();
    inputs.diverterVolts = diverterAppliedVolts.getValueAsDouble();
    inputs.feederRPM = feederVelocity.getValueAsDouble() * ConveyorConstants.FEEDER_GEAR_RATIO;
    inputs.currentAmps =
        new double[] {feederCurrent.getValueAsDouble(), diverterCurrent.getValueAsDouble()};
  }

  @Override
  public void stop() {
    feederMotor.set(0);
    diverterMotor.set(0);
  }

  @Override
  public void runPower(double power) {
    feederMotor.set(power);
    diverterMotor.set(power);
  }

  @Override
  public void runPower(double feederPower, double diverterPower) {
    feederMotor.set(feederPower);
    diverterMotor.set(diverterPower);
  }

  @Override
  public void setMotorMode(NeutralModeValue mode) {
    feederMotor.setNeutralMode(mode);
    diverterMotor.setNeutralMode(mode);
  }

  /**
   * Checks if there is something detected by the shooter side prox sensor
   *
   * @return if something is detected
   */
  private boolean detectedShooterSide() {
    return shooterProx.getVoltage() > ConveyorConstants.SHOOTER_PROX_THRESHOLD;
  }

  /**
   * Uses the prox sensor to determine if there is a note being stowed
   *
   * @return if there is a note in resting pos
   */
  private boolean noteInConveyor() {
    return detectedShooterSide();
  }
}
