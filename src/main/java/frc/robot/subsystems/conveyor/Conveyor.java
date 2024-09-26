package frc.robot.subsystems.conveyor;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/**
 * Nemesis Conveyor for 2024
 *
 * @author Ian Keller
 */
public class Conveyor extends SubsystemBase {
  // subsystem io
  private final ConveyorIO io;
  private final ConveyorIOInputsAutoLogged inputs = new ConveyorIOInputsAutoLogged();

  // TODO: update constants once we get the robot
  // tunable constants

  private double conveyorSpeed = .45;
  private double diverterSpeed = .5;

  // conveyor states
  public enum ConveyorStates {
    STOPPED,
    INTAKE,
    OUTTAKE,
    DIVERT,
    SHOOT,
    MANUAL
  }

  private ConveyorStates state = ConveyorStates.STOPPED;
  private double manualPower;

  /** Creates a new conveyor */
  public Conveyor(ConveyorIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    // handle inputs
    io.updateInputs(inputs);
    Logger.processInputs("Conveyor", inputs);

    // run the motors based on current state
    switch (state) {
      case STOPPED:
        io.stop();
        break;
      case INTAKE:
        if (inputs.hasNote) {
          io.stop();
        } else {
          io.runPower(conveyorSpeed);
        }
        break;
      case OUTTAKE:
        io.runPower(-conveyorSpeed);
        break;
      case DIVERT:
        io.runPower(-conveyorSpeed - .3, diverterSpeed);
        break;
      case SHOOT:
        io.runPower(conveyorSpeed);
        break;
      case MANUAL:
        io.runPower(manualPower);
        break;
      default:
        break;
    }
  }

  /** Stop the conveyor */
  public void setStopped() {
    state = ConveyorStates.STOPPED;
  }

  /** Intake piece into stow position */
  public void setIntaking() {
    state = ConveyorStates.INTAKE;
  }

  /** Outtake piece from front */
  public void setOuttaking() {
    state = ConveyorStates.OUTTAKE;
  }

  /** Moves piece into diverter (for amp/trap) */
  public void setDiverting() {
    state = ConveyorStates.DIVERT;
  }

  /** Move piece into shooter */
  public void setShooting() {
    state = ConveyorStates.SHOOT;
  }

  /**
   * Manual control of conveyor
   *
   * @param power - motor power, from -1.0 to 1.0
   */
  public void setManual(double power) {
    manualPower = power;
    state = ConveyorStates.MANUAL;
  }

  /**
   * Returns the current state of the conveyor
   *
   * @return current state
   */
  public ConveyorStates getState() {
    return state;
  }

  public boolean detectedShooterSide() {
    return inputs.detectedShooterSide;
  }

  /**
   * Uses the prox sensor to determine if there is a note being stowed
   *
   * @return if there is a note in resting pos
   */
  public boolean hasNote() {
    return inputs.hasNote;
  }

  public void setCoastMode(){
    io.setCoastMode();
  }
}

/**
 * spotless:off
 * autoformatter is a meanie, keeps breaking my easter eggs :(
 * 
 *                            _____  _____
 *                           <     `/     |
 *                            >          (
 *                           |   _     _  |
 *                           |  |_) | |_) |
 *                           |  | \ | |   |
 *                           |            |
 *            ______.______%_|            |__________  _____
 *          _/                                       \|     |
 *         |               A B H I K   R A Y                <
 *         |_____.-._________              ____/|___________|
 *                           |  2019-2023 |
 *                           | your code  |
 *                           | was useful |
 *                           |      once  |
 *                           |   _        <
 *                           |__/         |
 *                            / `--.      |
 *                          %|            |%
 *                      |/.%%|          -< @%%%
 *                      `\%`@|     v      |@@%@%%
 *                    .%%%@@@|%    |    % @@@%%@%%%%
 *               _.%%%%%%@@@@@@%%_/%\_%@@%%@@@@@@@%%%%%%
 * 
 * spotless:on
 */
