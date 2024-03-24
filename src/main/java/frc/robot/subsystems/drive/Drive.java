// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.util.LocalADStarAK;
import frc.robot.util.LoggedTunableNumber;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {
  private static final double MAX_LINEAR_SPEED = Units.feetToMeters(15.5);
  private static final double TRACK_WIDTH_X = Units.inchesToMeters(18.75);
  private static final double TRACK_WIDTH_Y = Units.inchesToMeters(18.75);
  // Uncomment for Jynx
  // static final double TRACK_WIDTH_X = Units.inchesToMeters(36);
  // static final double TRACK_WIDTH_Y = Units.inchesToMeters(36);
  private static final double DRIVE_BASE_RADIUS =
      Math.hypot(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0);
  private static final double MAX_ANGULAR_SPEED = MAX_LINEAR_SPEED / DRIVE_BASE_RADIUS;

  public static final Lock odometryLock = new ReentrantLock();
  private double[] lastModulePositionsMeters = new double[] {0.0, 0.0, 0.0, 0.0};
  public final GyroIO gyroIO;

  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR

  public final PIDController snapController = new PIDController(.34, 0.0, 0.0);
  public final PIDController noteController = new PIDController(.44, 0.0, .00001);
  public final PIDController linearMovementController = new PIDController(.44, 0.0, .00001);

  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
  private Pose2d pose = new Pose2d();
  private Rotation2d lastGyroRotation = new Rotation2d();

  public static LoggedTunableNumber snapControllermultiplier =
      new LoggedTunableNumber("SnapController/MaxSpeedRatio [0,1]", .5);
  public static LoggedTunableNumber noteControllermultiplier =
      new LoggedTunableNumber("NoteController/MaxSpeedRatio", 6);
  LoggedTunableNumber snapControllerP = new LoggedTunableNumber("SnapController/kP", .34);
  LoggedTunableNumber snapControllerD = new LoggedTunableNumber("SnapController/kD", .00001);
  LoggedTunableNumber snapControllerTolerance =
      new LoggedTunableNumber("SnapController/tolerance", .05);

  LoggedTunableNumber noteControllerP = new LoggedTunableNumber("NoteController/kP", .44);
  LoggedTunableNumber noteControllerD = new LoggedTunableNumber("NoteController/kD", .00001);
  LoggedTunableNumber noteControllerTolerance =
      new LoggedTunableNumber("NoteController/tolerance", .1);

  LoggedTunableNumber linearMovementControllerP =
      new LoggedTunableNumber("linearMovementController/kP", .44);
  LoggedTunableNumber linearMovementControllerD =
      new LoggedTunableNumber("linearMovementController/kD", .00001);
  LoggedTunableNumber linearMovementControllerTolerance =
      new LoggedTunableNumber("linearMovementController/tolerance", .1);

  // public static Drive getInstance(){
  //   return instance == null ? instance = new Drive() : instance;
  // }

  public Drive(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO) {
    if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red){
      gyroIO.setGyro(180);
    }
    this.gyroIO = gyroIO;
    modules[0] = new Module(flModuleIO, 0);
    modules[1] = new Module(frModuleIO, 1);
    modules[2] = new Module(blModuleIO, 2);
    modules[3] = new Module(brModuleIO, 3);

    snapController.setTolerance(snapControllerTolerance.get());
    snapController.enableContinuousInput(0, 2 * Math.PI);
    noteController.setTolerance(noteControllerTolerance.get());

    // Configure AutoBuilder for PathPlanner
    AutoBuilder.configureHolonomic(
        RobotContainer.poseEstimator::getLatestPose,
        RobotContainer.poseEstimator::resetPose,
        // this::getPose,
        // this::setPose,
        () -> kinematics.toChassisSpeeds(getModuleStates()),
        this::runVelocity,
        new HolonomicPathFollowerConfig(
            new PIDConstants(5, 0.0, 0),
            new PIDConstants(5.0, 0.0, 0.0),
            MAX_LINEAR_SPEED,
            DRIVE_BASE_RADIUS,
            new ReplanningConfig()),
        () ->
            DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == Alliance.Red,
        this);
    Pathfinding.setPathfinder(new LocalADStarAK());
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput(
              "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });
  }

  public void periodic() {
    odometryLock.lock(); // Prevents odometry updates while reading data
    gyroIO.updateInputs(gyroInputs);
    for (var module : modules) {
      module.updateInputs();
    }
    odometryLock.unlock();
    Logger.processInputs("Drive/Gyro", gyroInputs);
    for (var module : modules) {
      module.periodic();
    }

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
    }
    // Log empty setpoint states when disabled
    if (DriverStation.isDisabled()) {
      // Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      // Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    // Update odometry
    int deltaCount1 =
        gyroInputs.connected ? gyroInputs.odometryYawPositions.length : Integer.MAX_VALUE;
    for (int i = 0; i < 4; i++) {
      deltaCount1 = Math.min(deltaCount1, modules[i].getPositionDeltas().length);
    }
    for (int deltaIndex1 = 0; deltaIndex1 < deltaCount1; deltaIndex1++) {
      // Read wheel deltas from each module
      SwerveModulePosition[] wheelDeltas1 = new SwerveModulePosition[4];
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        wheelDeltas1[moduleIndex] = modules[moduleIndex].getPositionDeltas()[deltaIndex1];
      }

      // The twist represents the motion of the robot since the last
      // sample in x, y, and theta based only on the modules, without
      // the gyro. The gyro is always disconnected in simulation.
      Twist2d twist1 = kinematics.toTwist2d(wheelDeltas1);
      if (gyroInputs.connected) {
        // If the gyro is connected, replace the theta component of the twist
        // with the change in angle since the last sample.
        // Rotation2d gyroRotation1 = gyroInputs.odometryYawPositions[deltaIndex1];
        // twist1 =
        //     new Twist2d(twist1.dx, twist1.dy,
        // gyroRotation1.minus(lastGyroRotation).getRadians());
        // lastGyroRotation = gyroRotation1;
      }
      // Apply the twist (change since last sample) to the current pose

      pose = pose.exp(twist1);
    }

    // Update odometry
    // TODO: ANTHONY - For whatever reason, I cannot get the twsit1 above to work.
    // Someone should work on that
    SwerveModulePosition[] wheelDeltas = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      wheelDeltas[i] =
          new SwerveModulePosition(
              (modules[i].getPositionMeters() - lastModulePositionsMeters[i]),
              modules[i].getAngle());
      lastModulePositionsMeters[i] = modules[i].getPositionMeters();
    }
    var twist = kinematics.toTwist2d(wheelDeltas);
    var gyroYaw = new Rotation2d(gyroInputs.yawPosition.getRadians());
    if (gyroInputs.connected) {
      twist = new Twist2d(twist.dx, twist.dy, gyroYaw.minus(lastGyroRotation).getRadians());
    }
    lastGyroRotation = gyroYaw;

    // Delete the above and use original odometry code above and use twist1 -> twist
    RobotContainer.poseEstimator.addDriveData(Timer.getFPGATimestamp(), twist);
    updateTunableNumbers();
  }
  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisSpeeds speeds) {
    // Calculate module setpoints
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, MAX_LINEAR_SPEED);

    // Send setpoints to modules
    SwerveModuleState[] optimizedSetpointStates = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      // The module returns the optimized state, useful for logging
      optimizedSetpointStates[i] = modules[i].runSetpoint(setpointStates[i]);
    }

    // Log setpoint states
    // Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    // Logger.recordOutput("SwerveStates/SetpointsOptimized", optimizedSetpointStates);
  }

  /** Stops the drive. */
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = getModuleTranslations()[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  /** Runs forwards at the commanded voltage. */
  public void runCharacterizationVolts(double volts) {
    for (int i = 0; i < 4; i++) {
      modules[i].runCharacterization(volts);
    }
  }

  /** Returns the average drive velocity in radians/sec. */
  public double getCharacterizationVelocity() {
    double driveVelocityAverage = 0.0;
    for (var module : modules) {
      driveVelocityAverage += module.getCharacterizationVelocity();
    }
    return driveVelocityAverage / 4.0;
  }

  /** Returns the module states (turn angles and drive velocities) for all of the modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    // return pose.getRotation();
    return RobotContainer.poseEstimator.getLatestPose().getRotation();
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    // this.pose = pose;
    this.pose = RobotContainer.poseEstimator.getLatestPose();
  }

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    return MAX_LINEAR_SPEED;
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return MAX_ANGULAR_SPEED;
  }

  /** Returns an array of module translations. */
  public static Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
      new Translation2d(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0),
      new Translation2d(TRACK_WIDTH_X / 2.0, -TRACK_WIDTH_Y / 2.0),
      new Translation2d(-TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0),
      new Translation2d(-TRACK_WIDTH_X / 2.0, -TRACK_WIDTH_Y / 2.0)
    };
  }

  public void zeroGyro() {
    this.gyroIO.reset();
  }

  public Rotation2d getGyroYaw() {
    return this.gyroInputs.yawPosition;
  }

  private void updateTunableNumbers() {
    if (snapControllerP.hasChanged(hashCode())
        || snapControllerD.hasChanged(hashCode())
        || snapControllerTolerance.hasChanged(hashCode())) {
      snapController.setPID(snapControllerP.get(), 0.0, snapControllerD.get());
      snapController.setTolerance(snapControllerTolerance.get());
    }
    if (noteControllerP.hasChanged(hashCode())
        || noteControllerD.hasChanged(hashCode())
        || noteControllerTolerance.hasChanged(hashCode())) {
      noteController.setPID(noteControllerP.get(), 0.0, noteControllerD.get());
      noteController.setTolerance(noteControllerTolerance.get());
    }
  }

  public boolean snapControllerAtSetpoint() {
    Logger.recordOutput("SnapController/Error", snapController.getPositionError());
    return snapController.atSetpoint();
  }

  public double getSnapControllerError() {
    return snapController.getPositionError();
  }

  public ChassisSpeeds getCurrentChassisSpeeds() {
    SwerveModuleState[] currentStates = getModuleStates();
    ChassisSpeeds currentSpeeds = kinematics.toChassisSpeeds(currentStates);

    return currentSpeeds;
  }
}
