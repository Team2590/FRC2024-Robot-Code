package frc.robot.subsystems.elevatorarm;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.util.HelperFn;

import org.littletonrobotics.junction.AutoLogOutput;

public class Arm extends SubsystemBase {
  private ArmIOTalonFX arm = new ArmIOTalonFX();
  private ArmStates state;
  private double armSetpoint;
  private double tolerance = .001;
  private boolean requestHome = false;
  private boolean requestVertical = false;

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
    arm.updateTunableNumbers();
    arm.updateInputs(inputs);
    Logger.processInputs("Arm", inputs);
    switch (state) {
      case STOPPED:
        arm.stop();
        break;
      case MANUAL:
        // arm.armmanual();
        break;
      case APPROACHINGSETPOINT:
        arm.setPosition(armSetpoint);
        if (!HelperFn.isWithinTolerance(arm.armCancoder.getAbsolutePosition().getValueAsDouble(), armSetpoint, tolerance)) {
          state = ArmStates.APPROACHINGSETPOINT;
        } else {
          state = ArmStates.AT_SETPOINT;
        }
        break;
      case AT_SETPOINT:
        arm.setPosition(armSetpoint);
        if (HelperFn.isWithinTolerance(arm.armCancoder.getAbsolutePosition().getValueAsDouble(), armSetpoint, tolerance)) {
          if (requestHome){
            state = ArmStates.HOME;
          }
          else if (requestVertical){
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
  }

  /** Run open loop at the specified voltage. */
  public void setPosition(double setpoint) {
    armSetpoint = setpoint;
    state = ArmStates.APPROACHINGSETPOINT;
  }

  public void setHome(){
    requestHome = true;
    setPosition(.168);
  }

    public void setVertical(){
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
    return ArmStates.HOME;
  }
}
