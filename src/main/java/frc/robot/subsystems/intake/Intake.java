package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

/**
 * @author: Arnav Nayak, Shashank Madala
 */
public class Intake extends SubsystemBase {
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private double power = 1.0;
  private final IntakeIO io;
  private IntakeStates state;
  private AnalogInput intakeProx = new AnalogInput(IntakeConstants.INTAKE_PROX_CHANNEL);
  private boolean detectedNoteForAuto = false;

  public Intake(IntakeIO io) {
    this.io = io;
    state = IntakeStates.STOPPED;
  }

  public enum IntakeStates {
    STOPPED,
    INTAKE,
    OUTTAKE,
    MANUAL
  }

  @Override
  public void periodic() {
    // handle inputs
    io.updateInputs(inputs);
    // Logger.processInputs("Intake", inputs);
    // Logger.recordOutput("Intake/State", state);

    // run the motors based on current state
    switch (state) {
      case STOPPED:
        io.stop();
        break;
      case INTAKE:
        io.setPower(power);
        break;
      case OUTTAKE:
        io.setPower(-power);
        break;
      case MANUAL:
        // io.runPower(manualPower);
        break;
      default:
        System.out.println("Reached conveyor default case");
        break;
    }
    detectNoteForAuton();
  }

  public void setStopped() {
    state = IntakeStates.STOPPED;
  }

  public void setIntake() {
    state = IntakeStates.INTAKE;
  }

  public void setOutake() {
    state = IntakeStates.OUTTAKE;
  }

  public boolean detectNote() {
    if (intakeProx.getVoltage() > IntakeConstants.INTAKE_PROX_THRESHOLD) return true;
    else return false;
  }

  public boolean detectNoteForAuton() {
    if (!detectedNoteForAuto) {
      detectedNoteForAuto = detectNote();
    }
    return detectedNoteForAuto;
  }

  public void resetDetectedNoteForAuton() {
    detectedNoteForAuto = false;
  }

  public IntakeStates getState() {
    return state;
  }
}
