package frc.robot.subsystems.elevatorarm;

import com.ctre.phoenix6.controls.DutyCycleOut;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.util.HelperFn;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
  private ArmIOTalonFX arm = new ArmIOTalonFX();
  private ArmStates state;
  private double armSetpoint;
  private double tolerance = .01;
  private boolean requestHome = false;
  private boolean requestVertical = false;
  private DutyCycleOut power = new DutyCycleOut(0);

  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

  public static enum ArmStates {
    STOPPED,
    MANUAL,
    AT_SETPOINT,
    APPROACHINGSETPOINT,
    AMPTRAP, /*STOWED, */
    HOME
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
    System.out.println("request home: " + requestHome);
    arm.updateTunableNumbers();
    arm.updateInputs(inputs);
    Logger.processInputs("Arm", inputs);
    Logger.recordOutput("Arm/State", state);
    switch (state) {
      case STOPPED:
        arm.stop();
        break;
      case MANUAL:
        arm.setPower(power);
        break;
      case APPROACHINGSETPOINT:
        arm.setPosition(armSetpoint);
        if (!HelperFn.isWithinTolerance(
            arm.armCancoder.getAbsolutePosition().getValueAsDouble(), armSetpoint, tolerance)) {
          state = ArmStates.APPROACHINGSETPOINT;
        } else {
          state = ArmStates.AT_SETPOINT;
          // System.out.println("State is " + state);
        }
        break;
      case AT_SETPOINT:
        arm.setPosition(armSetpoint);
        if (HelperFn.isWithinTolerance(
            arm.armCancoder.getAbsolutePosition().getValueAsDouble(), armSetpoint, tolerance)) {
          state = ArmStates.AT_SETPOINT;
          if (requestHome) {
            state = ArmStates.HOME;
            requestHome = false;
          } else if (requestVertical) {
            state = ArmStates.AMPTRAP;
          }
        } else {
          state = ArmStates.APPROACHINGSETPOINT;
        }
        break;
      case AMPTRAP:
        requestVertical = false;
        break;
      case HOME:
        requestHome = false;
        break;
    }

    System.out.println("State is after periodic " + state);
  }

  /** Run open loop at the specified voltage. */
  public void setPosition(double setpoint) {
    armSetpoint = setpoint;
    if (!HelperFn.isWithinTolerance(inputs.armabspos, armSetpoint, tolerance)) {
      state = ArmStates.APPROACHINGSETPOINT;
    }
  }

  // public void setManualPower(double percent) {
  //   manualPower = percent;
  // }

  public void setHome() {
    requestHome = true;
    setPosition(.168);
  }

  public void setVertical() {
    requestVertical = true;
    setPosition(-2.0);
  }

  public void motionmagicintake() {
    state = ArmStates.HOME;
  }

  public void motionmagicamp() {
    state = ArmStates.AMPTRAP;
  }

  public void resetarm() {
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

  /** Run closed loop at the specified velocity. */
  public void runVelocity(double velocityRPM) {}

  /** Stops the flywheel. */
  public void setStopped() {
    state = ArmStates.STOPPED;
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

  public ArmStates getState() {
    // return state;
    return state;
  }
}
