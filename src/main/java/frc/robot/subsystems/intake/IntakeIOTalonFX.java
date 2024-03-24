package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants;

public class IntakeIOTalonFX implements IntakeIO {
  private TalonFX talon = new TalonFX(Constants.IntakeConstants.INTAKE_ID);
  private TalonFX follower = new TalonFX(Constants.IntakeConstants.INTAKE_FOLLOWER_ID);
  private final StatusSignal<Double> intakeLeaderPosition = talon.getPosition();
  private final StatusSignal<Double> intakeLeaderVelocity = talon.getVelocity();
  private final StatusSignal<Double> intakeLeaderAppliedVolts = talon.getMotorVoltage();
  private final StatusSignal<Double> intakeLeaderCurrent = talon.getStatorCurrent();

  private final StatusSignal<Double> intakeFollowerPosition = follower.getPosition();
  private final StatusSignal<Double> intakeFollowerVelocity = follower.getVelocity();
  private final StatusSignal<Double> intakeFollowerAppliedVolts = follower.getMotorVoltage();
  private final StatusSignal<Double> intakeFollowerCurrent = follower.getStatorCurrent();

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
    BaseStatusSignal.refreshAll();
    inputs.leaderVelocity = intakeLeaderVelocity.getValueAsDouble();
    inputs.leaderAppliedVolts = intakeLeaderAppliedVolts.getValueAsDouble();
    inputs.leaderCurrent = intakeLeaderCurrent.getValueAsDouble();
    inputs.leaderPosition = intakeLeaderPosition.getValueAsDouble();
    inputs.followerVelocity = intakeFollowerVelocity.getValueAsDouble();
    inputs.followerAppliedVolts = intakeFollowerAppliedVolts.getValueAsDouble();
    inputs.followerCurrent = intakeFollowerCurrent.getValueAsDouble();
    inputs.followerPosition = intakeFollowerPosition.getValueAsDouble();
  }
}
