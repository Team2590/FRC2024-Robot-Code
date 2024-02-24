package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants;

public class IntakeIOTalonFX implements IntakeIO {
  private TalonFX talon = new TalonFX(Constants.IntakeConstants.INTAKE_ID, Constants.CANBUS);
  private final StatusSignal<Double> intakePosition = talon.getPosition();
  private final StatusSignal<Double> intakeVelocity = talon.getVelocity();
  private final StatusSignal<Double> intakeAppliedVolts = talon.getMotorVoltage();
  private final StatusSignal<Double> intakeCurrent = talon.getStatorCurrent();

  @Override
  public void setPower(double velocity) {
    talon.set(velocity);
  }

  @Override
  public void stop() {
    talon.stopMotor();
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    BaseStatusSignal.refreshAll(intakePosition, intakeVelocity, intakeAppliedVolts, intakeCurrent);
    inputs.appliedVolts = intakeAppliedVolts.getValueAsDouble();
    inputs.current = intakeCurrent.getValueAsDouble();
    inputs.position = intakePosition.getValueAsDouble();
    inputs.velocity = intakeVelocity.getValueAsDouble();
  }
}
