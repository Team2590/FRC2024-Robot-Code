package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

/**
 * @author: Arnav Nayak, Shashank Madala
 */
public class Intake extends SubsystemBase {
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private LoggedTunableNumber intakePower = new LoggedTunableNumber("Intake/Power", .65);
  private double power = .65;
  private final IntakeIO io;
  private IntakeStates state;

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
    Logger.processInputs("Intake", inputs);

    // run the motors based on current state
    switch (state) {
      case STOPPED:
        io.stop();
        break;
      case INTAKE:
        io.setPower(intakePower.get());
        break;
      case OUTTAKE:
        io.setPower(-intakePower.get());
        break;
      case MANUAL:
        // io.runPower(manualPower);
        break;
      default:
        System.out.println("Reached conveyor default case");
        break;
    }
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
    return inputs.beamBreakDetected;
  }

  public IntakeStates getState() {
    return state;
  }
}
