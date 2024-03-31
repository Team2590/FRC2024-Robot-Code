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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.FieldConstants.Targets;
import frc.robot.Constants.LEDConstants;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.elevatorarm.Arm;
import frc.robot.subsystems.elevatorarm.Arm.ArmStates;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.flywheel.Flywheel.ShooterStates;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.nemesisLED.NemesisLED;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.LookupTable;
import org.littletonrobotics.junction.Logger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class Superstructure extends SubsystemBase {
  // TBD: declare variables to add subsystems into
  public static enum SuperstructureStates {
    DISABLED,
    RESET,
    IDLE,
    IDLE_INTAKING,
    IDLE_AMP,
    IDLE_PRIMING,
    IDLE_CLIMB,
    INTAKE,
    OUTTAKE,
    MANUAL_ARM,
    PRIMING_SHOOTER,
    PRIMED_SHOOTER,
    HAS_NOTE,
    SHOOT,
    SUBWOOFER_SHOT,
    PRIMING_AMP,
    SCORE_AMP,
    CLIMB,
    FLIPPING,
    SCORE_TRAP,
    ARM_CLIMB,
    FLING
  }

  private static enum IDLE_STATES {
    INTAKE,
    AMP,
    TRAP,
    CLIMB,
    DEFAULT
  }

  private SuperstructureStates systemState = SuperstructureStates.DISABLED;
  private IDLE_STATES idleState = IDLE_STATES.DEFAULT;
  private final Conveyor conveyor;
  private final Intake intake;
  private final Flywheel shooter;
  private final Arm arm;
  private final Climb climb;
  private final NemesisLED led;
  public boolean readyToShoot = false;
  private boolean climbed = false;
  private DutyCycleOut pwr = new DutyCycleOut(0);
  private final LoggedTunableNumber armAngle = new LoggedTunableNumber("Arm/Arm Angle", .168);
  private final LoggedTunableNumber offset = new LoggedTunableNumber("Arm/Arm offset", .01);
  private final LoggedTunableNumber flywheelSpeed =
      new LoggedTunableNumber("Flywheeel/Flywheel speed", Constants.ShooterConstants.SETPOINT);
  private final LookupTable armInterpolation;
  private final LookupTable armFlingInterpolation;
  private final LookupTable shooterflingInterpolation;

  /** The container for the robot. Pass in the appropriate subsystems from RobotContainer */
  public Superstructure(
      Conveyor conveyor, Intake intake, Flywheel shooter, Arm arm, Climb climb, NemesisLED led) {
    // assign args to local variables
    this.conveyor = conveyor;
    this.intake = intake;
    this.shooter = shooter;
    this.arm = arm;
    this.climb = climb;
    this.led = led;
    climb.resetRotationCount();

    // final double[] distance = {0, 1.599, 1.98, 2.67, 2.9, 3.48, 3.98, 4.6, 5.1, 5.698};
    // final double[] armSetpoint = {.168, .168, .135, .11, .09, 0.077, .069, 0.0625, 0.059, .055};
    final double[] distance = {
      1.29, 1.75, 2.05, 2.31, 2.65, 2.96, 3.25, 3.43, 3.7, 3.95, 4.26, 4.54, 4.73, 5.09, 5.4
    };
    final double[] armSetpoint = {
      .168, .14, .126, .11, .1, .095, .085, .082, .075, .071, .067, .064, .061, .058, .061
    };

    armInterpolation = new LookupTable(distance, armSetpoint);
    armFlingInterpolation =
        new LookupTable(
            Constants.FlingConstants.FLING_DISTANCE, Constants.FlingConstants.FLING_ARM_SETPOINT);
    shooterflingInterpolation =
        new LookupTable(
            Constants.FlingConstants.FLING_DISTANCE,
            Constants.FlingConstants.FLING_SHOOTER_SETPOINT);
  }

  /** This is where you would call all of the periodic functions of the subsystems. */
  @Override
  public void periodic() {
    Logger.recordOutput(
        "Pose/ErrorToSpeaker", RobotContainer.poseEstimator.currentErrorToSpeaker());
    Logger.recordOutput("Superstructure/climbed?", climbed);
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
        intake.setStopped();
        conveyor.setStopped();
        shooter.setStopped();
        arm.setHome();
        climb.setStopped();
        idleState = IDLE_STATES.DEFAULT;
        break;

      case IDLE:
        /*
         * Default state (No Button presses)
         * arm.setpositon(HOME) -- > HOME setpoint
         */
        climb.setStopped();
        if (conveyor.hasNote()) {
          led.off();
          if (arm.getState() == ArmStates.AT_SETPOINT
              && shooter.getState() == ShooterStates.AT_SETPOINT) {
          } else {
            intake.setStopped();
          }
        } else {
          conveyor.setStopped();
          shooter.setStopped();
          if (!climbed) {
            arm.setHome();
          }
          led.off();
        }
        break;
      case IDLE_INTAKING:
        if (intake.detectNote()) {
          led.setColor(LEDConstants.DETECT_NOTE_COLOR);
        }
        if (conveyor.hasNote()) {
          idleState = IDLE_STATES.DEFAULT;
          led.off();
          // intake.setStopped();
        }
        climb.setStopped();
        break;
      case IDLE_CLIMB:
        arm.setClimb();
        climb.setStopped();
        break;
      case IDLE_AMP:
        // Since the conveyor is moving towards one Prox sensor, using hasNote() should be
        // appropriate
        if (!conveyor.hasNote()) {
          idleState = IDLE_STATES.DEFAULT;
        }
        climb.setStopped();
        // if (climbed) { // aka if you're trying to score trap
        //   arm.setStopped();
        // }
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

        // if (arm.getState() == ArmStates.HOME) {
        if (intake.detectNote()) {
          led.setColor(LEDConstants.DETECT_NOTE_COLOR);
        }
        if (arm.getState() == ArmStates.HOME) {
          idleState = IDLE_STATES.INTAKE;
          intake.setIntake();
          conveyor.setIntaking();
        } else {
          arm.setHome();
        }
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
        break;
      case HAS_NOTE:
        intake.setStopped();
        break;
      case PRIMING_SHOOTER:
        /*
         * PRIMING_SHOOTER (For Auto Routines)
         * Run flywheel at desired velocity. Useful in auto routines.
         */
        shooter.shoot(flywheelSpeed.get());
        arm.setPosition(
            armInterpolation.getValue(
                    RobotContainer.poseEstimator.distanceToTarget(
                        Constants.FieldConstants.Targets.SPEAKER))
                + offset.get());
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
        if (shooter.getState() != ShooterStates.AT_SETPOINT
            || arm.getState() != ArmStates.AT_SETPOINT) {
          systemState = SuperstructureStates.PRIMING_SHOOTER;
        }
        break;

      case SHOOT:
        {
          double distanceToSpeaker =
              RobotContainer.poseEstimator.distanceToTarget(
                  Constants.FieldConstants.Targets.SPEAKER);
          Logger.recordOutput(
              "SnapController/Error", RobotContainer.getDrive().snapController.getPositionError());
          Logger.recordOutput(
              "Pose/ErrorToSpeaker", RobotContainer.poseEstimator.currentErrorToSpeaker());
          double armDistanceSetPoint =
              armInterpolation.getValue(
                  RobotContainer.poseEstimator.distanceToTarget(
                      Constants.FieldConstants.Targets.SPEAKER));
          Logger.recordOutput("Arm/DistanceSetpoint", armDistanceSetPoint);
          arm.setPosition(armDistanceSetPoint);
          shooter.shoot(flywheelSpeed.get());
          if (DriverStation.isAutonomousEnabled()) {
            DriveCommands.snapToTargetForAuto(
                RobotContainer.getDrive(), () -> 0, () -> 0, Targets.SPEAKER);
          }
          if (arm.getState() == ArmStates.AT_SETPOINT
              && shooter.getState() == ShooterStates.AT_SETPOINT
              && (Math.abs(RobotContainer.poseEstimator.currentErrorToSpeaker()) < .05)) {
            conveyor.setShooting();
            // Since the conveyor is moving towards one Prox sensor, using hasNote() should be
            // appropriate
            if (!conveyor.hasNote()) {
              idleState = IDLE_STATES.DEFAULT;
            }
          }

          /*
           * SHOOT (Right Driver Trigger)
           * if shooter and arm are PRIMED, conveyor moves note and shoots
           */
          // if (systemState == SuperstructureStates.PRIMED_SHOOTER) {
          //   conveyor.setShooting();
          // }
          break;
        }
      case SUBWOOFER_SHOT:
        arm.setPosition(ArmConstants.HOME_SETPOINT);
        shooter.shoot(Constants.ShooterConstants.SETPOINT);
        if (shooter.getState() == ShooterStates.AT_SETPOINT) {
          conveyor.setShooting();
          idleState = IDLE_STATES.DEFAULT;
        }
        break;

      case PRIMING_AMP:
        /*
         * PRIMING_AMP
         * Moves arm to AMP setpoint
         */
        // arm.setposition(AMP);
        idleState = IDLE_STATES.AMP;
        if (climbed) {
          // :) - aheulitt
          led.setRainbow();
          arm.setPosition(ArmConstants.TRAP_SETPOINT);
        } else {
          arm.setPosition(ArmConstants.AMP_SETPOINT);
        }
        break;

      case SCORE_AMP:
        /*
         * PRIMED_AMP
         * Arm is at AMP Setpoint -- > conveyor diverts to score AMP
         */
        if (arm.getState() == ArmStates.AT_SETPOINT) {
          conveyor.setDiverting();
          if (!conveyor.hasNote()) {
            idleState = IDLE_STATES.DEFAULT;
          }
        }
        break;
      case SCORE_TRAP:
        arm.setPosition(ArmConstants.TRAP_SETPOINT);
        if (arm.getState() == ArmStates.AT_SETPOINT) {
          conveyor.setDiverting();
          idleState = IDLE_STATES.DEFAULT;
        }
        break;
      case ARM_CLIMB:
        arm.setPosition(ArmConstants.TRAP_SETPOINT);
        break;
      case CLIMB:
        /*
         * arm.setposition(HOME); -- > Stow the arm for climb
         * set system state to IDLE before climbing action ? (TBD)
         * climb.climb() -- > Sets climb to manual state
         */
        climb.run();
        climbed = true;
        idleState = IDLE_STATES.CLIMB;
        break;
      case FLIPPING:
        climb.flip();
        climbed = true;
        idleState = IDLE_STATES.CLIMB;
        break;
      case FLING:
        if (DriverStation.getAlliance().isPresent()) {
          var flingPose =
              DriverStation.getAlliance().get() == Alliance.Blue
                  ? Constants.FlingConstants.BLUE_FLING_POSE
                  : Constants.FlingConstants.RED_FLING_POSE;
          Logger.recordOutput(
              "Pose/FlingDistance",
              RobotContainer.poseEstimator.distanceBetweenPoses(
                  RobotContainer.poseEstimator.getLatestPose(), flingPose));
          double armDistanceSetPoint =
              armFlingInterpolation.getValue(
                  RobotContainer.poseEstimator.distanceBetweenPoses(
                      RobotContainer.poseEstimator.getLatestPose(), flingPose));
          double shooterSetPoint =
              shooterflingInterpolation.getValue(
                  RobotContainer.poseEstimator.distanceBetweenPoses(
                      RobotContainer.poseEstimator.getLatestPose(), flingPose));
          arm.setPosition(armAngle.get());
          shooter.shoot(flywheelSpeed.get());
          if (arm.getState() == ArmStates.AT_SETPOINT
              && shooter.getState() == ShooterStates.AT_SETPOINT) {
            conveyor.setShooting();
          }
        }
        break;
    }
    Logger.recordOutput("Superstructure/State", systemState);
    Logger.recordOutput("Superstructure/ArmState", arm.getState());
    Logger.recordOutput("Superstructure/ShooterState", shooter.getState());
    Logger.recordOutput("Superstructure/IntakeState", intake.getState());
    Logger.recordOutput("Superstructure/ConveyorState", conveyor.getState());
    Logger.recordOutput(
        "Odometry/DistanceToTarget",
        RobotContainer.poseEstimator.distanceToTarget(Constants.FieldConstants.Targets.SPEAKER));
  }

  public void stop() {
    systemState = SuperstructureStates.DISABLED;
  }

  public void idle() {
    if (idleState != IDLE_STATES.AMP && idleState == IDLE_STATES.CLIMB) {
      systemState = SuperstructureStates.IDLE_CLIMB;
    } else if (idleState == IDLE_STATES.INTAKE) {
      systemState = SuperstructureStates.IDLE_INTAKING;
    } else if (idleState == IDLE_STATES.AMP) {
      systemState = SuperstructureStates.IDLE_AMP;
    } else {
      systemState = SuperstructureStates.IDLE;
    }
  }

  public void intake() {
    systemState = SuperstructureStates.INTAKE;
  }

  public void resetRobot() {
    climbed = false;
    systemState = SuperstructureStates.RESET;
  }

  public boolean note_present() {
    return conveyor.hasNote();
  }

  public void primeShooter() {
    if (DriverStation.isAutonomousEnabled()) {
      if (note_present()) {
        shooter.shoot(flywheelSpeed.get());
      } else {
        intake();
      }
    } else {
      shooter.shoot(flywheelSpeed.get());
    }
  }

  public void primeAmp() {
    systemState = SuperstructureStates.PRIMING_AMP;
  }

  public void hasNote() {
    systemState = SuperstructureStates.HAS_NOTE;
  }

  public void shoot() {
    systemState = SuperstructureStates.SHOOT;
  }

  public void shoot(double setpoint) {
    systemState = SuperstructureStates.SHOOT;
  }

  public void stopShooter() {
    shooter.setStopped();
  }

  public void subwooferShot() {
    systemState = SuperstructureStates.SUBWOOFER_SHOT;
  }

  /**
   * Returns true if we are in the process of shooting i.e either priming up to shooting or
   * shooting.
   */
  public boolean isShooting() {
    return systemState == SuperstructureStates.SHOOT;
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
    if (arm.getAbsolutePosition() <= ArmConstants.ARM_MAX) {
      arm.setPosition(ArmConstants.ARM_MAX);
    } else {
      pwr = new DutyCycleOut(-0.1);
      systemState = SuperstructureStates.MANUAL_ARM;
    }
  }

  public void armDown() {
    if (arm.getAbsolutePosition() >= ArmConstants.HOME_SETPOINT) {
      arm.setPosition(ArmConstants.HOME_SETPOINT);
    }
    pwr = new DutyCycleOut(0.1);
    systemState = SuperstructureStates.MANUAL_ARM;
  }

  public void armClimb() {
    systemState = SuperstructureStates.ARM_CLIMB;
  }

  public void flip() {
    systemState = SuperstructureStates.FLIPPING;
  }

  public void runIntake() {
    intake.setIntake();
    // conveyor.setIntaking();
  }

  public void fling() {
    systemState = SuperstructureStates.FLING;
  }

  public SuperstructureStates getState() {
    return systemState;
  }

  public void runConveyor() {
    conveyor.setManual(.25);
    ;
  }

  public void stopConveyor() {
    conveyor.setStopped();
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
