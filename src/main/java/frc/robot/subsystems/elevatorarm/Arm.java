package frc.robot.subsystems.elevatorarm;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  private ArmIOTalonFX arm = new ArmIOTalonFX();
  private States state;
  

  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
  public static enum States {
        STOPPED, MANUAL, HOLDSETPOINT, APPROACHINGSETPOINT, AMPTRAP, /*STOWED, */INTAKE
    }

  /** Creates a new Flywheel. */
  public Arm() {
    state = States.STOPPED;
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
                arm.armmanual();
                break;
            case APPROACHINGSETPOINT:
                arm.setmotionmagic();
                break;
            case HOLDSETPOINT:
                arm.setmotionmagic();
                break;
            // case STOWED:
            //     arm.setmotionmagicstow();
            //     break;
            case AMPTRAP:
                arm.setmotionmagicamp();
                break;
            case INTAKE:
                arm.setmotionmagicintake();
                break;
            

        }
  }

  /** Run open loop at the specified voltage. */
  public void motionmagic1() {
    state = States.APPROACHINGSETPOINT;
  }

  public void motionmagicintake() {
    state = States.INTAKE;
  }

  public void motionmagicamp() {
    state = States.AMPTRAP;
  }

  public void resetarm() {
    arm.resetArm();
  }

  /** Run closed loop at the specified velocity. */
  public void runVelocity(double velocityRPM) {}

  /** Stops the flywheel. */
  public void stop() {
    state = States.STOPPED;
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
}