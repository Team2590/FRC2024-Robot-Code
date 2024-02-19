package frc.robot.subsystems.elevatorarm;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;

public class Arm extends SubsystemBase {
  private ArmIOTalonFX arm = new ArmIOTalonFX();
  private ArmStates state;

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
    // arm.updateTunableNumbers();
    // arm.updateInputs(inputs);
    // Logger.processInputs("Arm", inputs);
    // switch (state) {
    //   case STOPPED:
    //     // arm.stop();
    //     break;
    //   case MANUAL:
    //     arm.armmanual();
    //     break;
    //   case APPROACHINGSETPOINT:
    //     arm.setmotionmagic();
    //     if (arm.atsetpoint()) {
    //       state = ArmStates.AT_SETPOINT;

    //     } else {
    //       state = ArmStates.APPROACHINGSETPOINT;
    //     }
    //     break;
    //   case AT_SETPOINT:
    //     arm.setmotionmagic();
    //     if (arm.atsetpoint()) {
    //       state = ArmStates.AT_SETPOINT;

    //     } else {
    //       state = ArmStates.APPROACHINGSETPOINT;
    //     }
    //     break;
    //     // case STOWED:
    //     //     arm.setmotionmagicstow();
    //     //     break;
    //   case AMPTRAP:
    //     arm.setmotionmagicamp();
    //     break;
    //   case HOME:
    //     arm.setmotionmagicintake();
    //     break;
    // }
  }

  /** Run open loop at the specified voltage. */
  public void setPosition() {
    arm.setmotionmagicintake();
    state = ArmStates.APPROACHINGSETPOINT;
  }

  public void motionmagicintake() {
    state = ArmStates.HOME;
  }

  public void motionmagicamp() {
    state = ArmStates.AMPTRAP;
  }

  public void armmanualup() {
    state = ArmStates.MANUAL;
    arm.armmanualup();
  }

  public void armmanualdown() {
    state = ArmStates.MANUAL;
    arm.armmanualdown();
  }

  public void resetarm() {
    arm.resetArm();
  }

  public void atsetpoint() {
    if (arm.atsetpoint()) {
      state = ArmStates.AT_SETPOINT;
    } else {
      state = ArmStates.APPROACHINGSETPOINT;
    }
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
