package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

/**
 * @author: Arnav Nayak, Shashank Madala
 */
public class Intake extends SubsystemBase {
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  LoggedTunableNumber power = new LoggedTunableNumber("Intake/PowerPercent", .5);
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
    Logger.processInputs("Conveyor", inputs);
    Logger.recordOutput("Intake/State", state);

    // run the motors based on current state
    switch (state) {
      case STOPPED:
        io.stop();
        break;
      case INTAKE:
        io.setPower(power.get());
        break;
      case OUTTAKE:
        io.setPower(-power.get());
        break;
      case MANUAL:
        // io.runPower(manualPower);
        break;
      default:
        System.out.println("Reached conveyor default case");
        break;
    }
  }

  public void setStop() {
    state = IntakeStates.STOPPED;
  }

  public void setIntake() {
    state = IntakeStates.INTAKE;
  }

  public void setOutake() {
    state = IntakeStates.OUTTAKE;
  }

  public IntakeStates getState() {
    return state;
  }
}