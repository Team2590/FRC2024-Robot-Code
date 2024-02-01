package frc.robot.subsystems.climb;

import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants;

public class ClimbIOTalonFX implements ClimbIO {
  private final TalonFX left_motor = new TalonFX(20);
  // private final TalonFX right_motor = new TalonFX(200);

  public final double maxPosition =
      ((Constants.CLIMBER_MAX_HEIGHT) / (Math.PI * Constants.CLIMBER_WHEEL_RADIUS))
          * Constants.CLIMBER_GEAR_RATIO;

  private final double speed = 0.2;

  // public enum Limits {
  //   TOO_HIGH,
  //   TOO_LOW,
  //   NONE
  // }

  // LoggedTunableNumber kP = new LoggedTunableNumber("kP", 0.0);
  // LoggedTunableNumber kD = new LoggedTunableNumber("kD", 0.0);
  // LoggedTunableNumber kFF = new LoggedTunableNumber("kFF", 0.0);

  // private TalonFXConfiguration config;

  public ClimbIOTalonFX() {
    // config.Slot0.kP = kP.get();
    // config.Slot0.kD = kD.get();
    // config.Slot0.kV = kFF.get();
    // left_motor.getConfigurator().apply(config);
    // config.
    // right_motor.getConfigurator().apply(config);
    // right_motor.setControl(new Follower(left_motor.getDeviceID(), false));
    // left_motor.optimizeBusUtilization();
    // right_motor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ClimbIOInputs inputs) {
    inputs.positionRotations = left_motor.getPosition().getValueAsDouble();
  }

  public void up() {
    if (left_motor.getPosition().getValueAsDouble() > maxPosition) {
      left_motor.stopMotor();
    } else {
      left_motor.set(speed);
    }
  }

  public void down() {
    if (left_motor.getPosition().getValueAsDouble() < 0) {
      left_motor.stopMotor();
    } else {
      left_motor.set(-speed);
    }
  }

  public void stop() {
    left_motor.stopMotor();
  }

  public void reset() {
    down();
  }

  public double getPosition() {
    return left_motor.getPosition().getValueAsDouble();
  }

  public void resetRotationCount() {
    left_motor.setPosition(0);
  }

  // public void updateTunableNumbers() {
  //     if (kP.hasChanged(0) || kD.hasChanged(0) || kFF.hasChanged(0)) {
  //         config.Slot0.kP = kP.get();
  //         config.Slot0.kD = kD.get();
  //         config.Slot0.kV = kFF.get();
  //         left_motor.getConfigurator().apply(config);
  //         right_motor.getConfigurator().apply(config);
  //     }
  // }
}
