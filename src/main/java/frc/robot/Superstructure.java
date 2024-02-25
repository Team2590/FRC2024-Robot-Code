// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.elevatorarm.Arm;
import frc.robot.subsystems.elevatorarm.Arm.ArmStates;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.flywheel.Flywheel.ShooterStates;
import frc.robot.subsystems.intake.Intake;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.LookupTable;
import org.littletonrobotics.junction.Logger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class Superstructure {
  // TBD: declare variables to add subsystems into
  public static enum SuperstructureStates {
    DISABLED,
    RESET,
    IDLE,
    INTAKE,
    OUTTAKE,
    MANUAL_ARM,
    PRIMING_SHOOTER,
    PRIMED_SHOOTER,
    HAS_NOTE,
    SHOOT,
    PRIMING_AMP,
    SCORE_AMP,
    CLIMB
  }

  private final TalonFX leader = new TalonFX(24, Constants.CANBUS);
  private final TalonFX follower = new TalonFX(25, Constants.CANBUS);
  private SuperstructureStates systemState = SuperstructureStates.DISABLED;
  private final Conveyor conveyor;
  private final Intake intake;
  private final Flywheel shooter;
  private final Arm arm;
  public boolean readyToShoot = false;
  private DutyCycleOut pwr = new DutyCycleOut(0);
  private final LoggedTunableNumber armAngle = new LoggedTunableNumber("Arm/Arm Angle", 0);
  private final LoggedTunableNumber flywheelSpeedInput =
      new LoggedTunableNumber("Flywheel/Flywheel Speed", 2300.0);
  private final LookupTable armInterpolation;

  /** The container for the robot. Pass in the appropriate subsystems from RobotContainer */
  public Superstructure(Conveyor conveyor, Intake intake, Flywheel shooter, Arm arm) {
    // assign args to local variables
    this.conveyor = conveyor;
    this.intake = intake;
    this.shooter = shooter;
    this.arm = arm;

    final double[] distance = {
      0, .9144, 1.2192, 1.524, 1.8288, 2.1336, 2.4384, 2.7432, 3.048, 3.3528, 3.6576, 3.9624,
      4.2672, 4.572
    };
    final double[] armSetpoint = {
      .16, .16, .145, .133, .12, .109, .103, .097, .090, .09, .088, .084, .08, .076
    };

    armInterpolation = new LookupTable(distance, armSetpoint);
  }

  /** This is where you would call all of the periodic functions of the subsystems. */
  public void periodic() {
    Logger.recordOutput("Arm/RangeTOTarget", RobotContainer.poseEstimator.distanceToSpeaker());
    switch (systemState) {
      case DISABLED:
        // stop
        intake.setStopped();
        conveyor.setStopped();
        shooter.setStopped();
        // arm.setStopped();
        break;

      case RESET:
        /*
         * TBD -- > Simmilar to IDLE state ?
         */
        // C:\Users\Nemesis\Documents\2024\FRC2024-Robot-Code\src\main\java\frc\robot\autos\StartPathCommand.java
        break;

      case IDLE:
        /*
         * Default state (No Button presses)
         * arm.setpositon(HOME) -- > HOME setpoint
         */
        // arm.setPosition();
        shooter.setStopped();
        intake.setStopped();
        conveyor.setStopped();
        arm.setHome();
        leader.stopMotor();
        follower.stopMotor();
        break;
      case MANUAL_ARM:
        arm.manual(pwr);
        break;
      case INTAKE:
        /*
         * INTAKE (On left Driver trigger)
         * Gets subsytems ready for intaking
         * If conveyor.hasNote is true :
         * Stop intake && transition to HAS_NOTE state
         */
        arm.setHome();
        // if (arm.getState() == ArmStates.HOME) {
        intake.setIntake();
        conveyor.setIntaking();
        // }
        if (conveyor.hasNote()) {
          intake.setStopped();
          systemState = SuperstructureStates.HAS_NOTE;
        }
        break;

      case OUTTAKE:
        /*
         * OUTTAKE (On left Driver trigger)
         * Gets subsytems ready for intaking
         * If conveyor.hasNote is true :
         * Stop intake && transition to HAS_NOTE state
         */
        arm.setHome();
        intake.setOutake();
        conveyor.setOuttaking();
        break;
      case HAS_NOTE:
        // EMPTY STATE -- > "Helper Transition" to Speaker shooting || AMP/TRAP
        break;
      case PRIMING_SHOOTER:
        /*
         * PRIMING_SHOOTER (For Auto Routines)
         * Run flywheel at desired velocity. Useful in auto routines.
         */
        shooter.shoot(flywheelSpeedInput.get());
        // Don't need any transition here, we want to stay in this state
        // until SHOOT is called.
        break;

      case PRIMED_SHOOTER:
        /*
         * PRIMED_SHOOTER
         * If flywheel and arm are not ready to shoot, go back to PRIMING state to
         * adjust
         */
        // arm.getState() != ArmStates.AT_SETPOINT
        if (shooter.getState() != ShooterStates.AT_SETPOINT) {
          systemState = SuperstructureStates.PRIMING_SHOOTER;
        }
        break;

      case SHOOT:
        double armDistanceSetPoint =
            armInterpolation.getValue(
                RobotContainer.poseEstimator.distanceToSpeaker() + Units.inchesToMeters(15));
        Logger.recordOutput("Arm/DistanceSetpoint", armDistanceSetPoint);
        arm.setPosition(armDistanceSetPoint);
        shooter.shoot(flywheelSpeedInput.get());
        if (arm.getState() == ArmStates.AT_SETPOINT
            && shooter.getState() == ShooterStates.AT_SETPOINT) {
          conveyor.setShooting();
        }

        /*
         * SHOOT (Right Driver Trigger)
         * if shooter and arm are PRIMED, conveyor moves note and shoots
         */
        // if (systemState == SuperstructureStates.PRIMED_SHOOTER) {
        //   conveyor.setShooting();
        // }
        break;

      case PRIMING_AMP:
        /*
         * PRIMING_AMP
         * Moves arm to AMP setpoint
         */
        // arm.setposition(AMP);
        break;

      case SCORE_AMP:
        /*
         * PRIMED_AMP
         * Arm is at AMP Setpoint -- > conveyor diverts to score AMP
         */
        arm.setPosition(-.258);
        if (arm.getState() == ArmStates.AT_SETPOINT) {
          conveyor.setDiverting();
        }

        break;

      case CLIMB:
        /*
         * arm.setposition(HOME); -- > Stow the arm for climb
         * set system state to IDLE before climbing action ? (TBD)
         * climb.climb() -- > Sets climb to manual state
         */
        leader.set(.25);
        follower.set(-.25);
        break;
    }
    Logger.recordOutput("Superstructure/State", systemState);
    Logger.recordOutput("Superstructure/ArmState", arm.getState());
    Logger.recordOutput("Superstructure/ShooterState", shooter.getState());
    Logger.recordOutput("Superstructure/IntakeState", intake.getState());
    Logger.recordOutput("Superstructure/ConveyorState", conveyor.getState());
  }

  public void stop() {
    systemState = SuperstructureStates.DISABLED;
  }

  public void idle() {
    systemState = SuperstructureStates.IDLE;
  }

  public void intake() {
    systemState = SuperstructureStates.INTAKE;
  }

  public void primeShooter() {
    // systemState = SuperstructureStates.PRIMING_SHOOTER;
    shooter.shoot(flywheelSpeedInput.get());
  }

  public void hasNote() {
    systemState = SuperstructureStates.HAS_NOTE;
  }

  public void shoot() {
    systemState = SuperstructureStates.SHOOT;
  }

  /**
   * Returns true if we are in the process of shooting i.e either priming up to shooting or
   * shooting.
   */
  public boolean isShooting() {
    return systemState == SuperstructureStates.SHOOT;
  }

  public void primingAmp() {
    systemState = SuperstructureStates.PRIMING_AMP;
  }

  public void scoreAmp() {
    systemState = SuperstructureStates.SCORE_AMP;
  }

  public void climb() {
    systemState = SuperstructureStates.CLIMB;
  }

  public void outtake() {
    systemState = SuperstructureStates.OUTTAKE;
  }

  public void armUp() {
    // optimization, make these two duty cycles local variables so these aren't created each time
    pwr = new DutyCycleOut(-0.1);
    systemState = SuperstructureStates.MANUAL_ARM;
  }

  public void armDown() {
    pwr = new DutyCycleOut(0.1);
    systemState = SuperstructureStates.MANUAL_ARM;
  }

  public SuperstructureStates getState() {
    return systemState;
  }

  public Arm getArm() {
    return arm;
  }

  public Conveyor getConveyor() {
    return conveyor;
  }

  public Intake getIntake() {
    return intake;
  }

  public Flywheel getShooter() {
    return shooter;
  }
}
