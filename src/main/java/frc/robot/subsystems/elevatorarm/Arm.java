package frc.robot.subsystems.elevatorarm;

import com.ctre.phoenix6.controls.DutyCycleOut;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.util.HelperFn;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
  private ArmIOTalonFX arm = new ArmIOTalonFX();
  private ArmStates state;
  private double armSetpoint;
  private double tolerance = .005;
  private DutyCycleOut power = new DutyCycleOut(0);
  private LoggedTunableNumber homeSetpoint = new LoggedTunableNumber("Arm/homeSetpoint", .168);
  private double targetHome = ArmConstants.HOME_SETPOINT;
  private boolean isClimbing = false;

  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

  public static enum ArmStates {
    STOPPED,
    MANUAL,
    AT_SETPOINT,
    APPROACHINGSETPOINT,
    AMPTRAP, /*STOWED, */
    APPROACHING_HOME,
    HOME,
  }

  /**
   * Creates a new Flywheel.
   *
   * @param armIOTalonFX
   */
  public Arm(ArmIOTalonFX armIOTalonFX) {
    state = ArmStates.STOPPED;
  }

  @Override
  public void periodic() {
    arm.updateTunableNumbers();
    arm.updateInputs(inputs);
    Logger.processInputs("Arm", inputs);
    switch (state) {
      case STOPPED:
        arm.stop();
        break;
      case MANUAL:
        arm.setPower(power);
        break;
      case APPROACHINGSETPOINT:
        arm.setPosition(armSetpoint);
        if (!isArmAtSetPointPosition(armSetpoint)) {
          state = ArmStates.APPROACHINGSETPOINT;
        } else {
          state = ArmStates.AT_SETPOINT;
        }
        break;
      case AT_SETPOINT:
        if (isArmAtSetPointPosition(armSetpoint)) {
          state = ArmStates.AT_SETPOINT;
          if (!isClimbing && armSetpoint == ArmConstants.HOME_SETPOINT) {
            state = ArmStates.HOME;
          }
        } else {
          state = ArmStates.APPROACHINGSETPOINT;
        }
        break;
      case AMPTRAP:
        break;
      case APPROACHING_HOME:
        arm.setPosition(ArmConstants.HOME_SETPOINT);
        if (isArmAtSetPointPosition(ArmConstants.HOME_SETPOINT)) {
          state = ArmStates.HOME;
        }
      case HOME:
        if (!isArmAtSetPointPosition(ArmConstants.HOME_SETPOINT)) {
          state = ArmStates.APPROACHING_HOME;
        }
        break;
    }
  }

  private boolean isArmAtSetPointPosition(double setPoint) {
    return HelperFn.isWithinTolerance(
        arm.armCancoder.getAbsolutePosition().getValueAsDouble(), setPoint, tolerance);
  }

  /** Run open loop at the specified voltage. */
  public void setPosition(double setpoint) {
    if (setpoint <= ArmConstants.ARM_MAX) {
      setpoint = ArmConstants.ARM_MAX;
    } else if (setpoint >= ArmConstants.CLIMB_SETPOINT) {
      setpoint = ArmConstants.CLIMB_SETPOINT;
    }
    armSetpoint = setpoint;
    if (!HelperFn.isWithinTolerance(inputs.armabspos, armSetpoint, tolerance)) {
      state = ArmStates.APPROACHINGSETPOINT;
    } else {
      state = ArmStates.AT_SETPOINT;
    }
  }

  // public void setManualPower(double percent) {
  //   manualPower = percent;
  // }

  public void setHome() {
    if (isArmAtSetPointPosition(ArmConstants.HOME_SETPOINT)) {
      state = ArmStates.HOME;
    } else {
      state = ArmStates.APPROACHING_HOME;
    }
  }

  public void setClimb() {
    isClimbing = true;
    if (!isArmAtSetPointPosition(ArmConstants.CLIMB_SETPOINT)) {
      setPosition(ArmConstants.CLIMB_SETPOINT);
    }
  }

  public void resetarm() {
    isClimbing = false;
    arm.resetArm();
  }

  public void manual(DutyCycleOut request) {
    power = request;
    state = ArmStates.MANUAL;
  }

  public void armmanualdown() {
    state = ArmStates.MANUAL;
    DutyCycleOut power = new DutyCycleOut(0.1);
    arm.setPower(power);
  }

  /** Stops the flywheel. */
  public void setStopped() {
    state = ArmStates.STOPPED;
  }

  public double getAbsolutePosition() {
    return arm.armCancoder.getAbsolutePosition().getValueAsDouble();
  }

  public ArmStates getState() {
    return state;
  }
}
