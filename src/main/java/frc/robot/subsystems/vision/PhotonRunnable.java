package frc.robot.subsystems.vision;

import static frc.robot.Constants.FieldConstants.FIELD_LENGTH_METERS;
import static frc.robot.Constants.FieldConstants.FIELD_WIDTH_METERS;
import static frc.robot.Constants.VisionConstants.APRILTAG_AMBIGUITY_THRESHOLD;
import static frc.robot.Constants.VisionConstants.CAMERA_HEIGHT_METERS;
import static frc.robot.Constants.VisionConstants.CAMERA_PITCH;
import static frc.robot.Constants.VisionConstants.CAMERA_ROLL;
import static frc.robot.Constants.VisionConstants.CAMERA_YAW;
import static frc.robot.Constants.VisionConstants.RobotToCam;

import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.RobotContainer;
import frc.util.PoseEstimator.TimestampedVisionUpdate;
import java.util.ArrayList;
import java.util.concurrent.atomic.AtomicReference;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/** Runnable that gets AprilTag data from PhotonVision. */
public class PhotonRunnable implements Runnable {

  private final PhotonPoseEstimator photonPoseEstimator;
  private final PhotonCamera photonCamera;
  private final AtomicReference<EstimatedRobotPose> atomicEstimatedRobotPose =
      new AtomicReference<EstimatedRobotPose>();
  public final ArrayList<TimestampedVisionUpdate> updates =
      new ArrayList<TimestampedVisionUpdate>();
  private static double distanceToSpeaker = 0;
  private static Pose3d RobotPose = new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0));
  private static PhotonPipelineResult photonResults;
  private Transform3d cameraTransform;

  public PhotonRunnable(String name, Transform3d cameraTransform3d) {
    this.photonCamera = new PhotonCamera(name);
    this.cameraTransform = cameraTransform3d;
    PhotonPoseEstimator photonPoseEstimator = null;
    var layout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    // PV estimates will always be blue, they'll get flipped by robot thread
    layout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
    if (photonCamera != null) {
      photonPoseEstimator =
          new PhotonPoseEstimator(
              layout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, photonCamera, cameraTransform3d);
      photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }
    this.photonPoseEstimator = photonPoseEstimator;
  }

  private static final double[] tagHeights = {
    0, 1.36, 1.36, 1.45, 1.45, 1.36, 1.36, 1.45, 1.45, 1.36, 1.36, 1.32, 1.32, 1.32, 1.32, 1.32,
    1.32
  };

  public Object stringify(Object o) {
    if (o == null) {
      return "null";
    } else {
      return o.toString();
    }
  }

  @Override
  public void run() {
    // Get AprilTag data
    if (photonPoseEstimator != null && photonCamera != null && !RobotState.isAutonomous()) {
      photonResults = photonCamera.getLatestResult();
      var timestamp = photonResults.getTimestampSeconds();
      if (photonResults.hasTargets()) {
        if (photonResults.targets.get(0).getPoseAmbiguity() < APRILTAG_AMBIGUITY_THRESHOLD) {
          for( PhotonTrackedTarget target : photonResults.getTargets()){
            if ((DriverStation.getAlliance().get() == Alliance.Red && target.getFiducialId() == 4) || (DriverStation.getAlliance().get() == Alliance.Blue && target.getFiducialId() == 7 )){
              distanceToSpeaker =
                  PhotonUtils.calculateDistanceToTargetMeters(
                      this.cameraTransform.getZ(),
                      tagHeights[photonResults.getBestTarget().getFiducialId()],
                      this.cameraTransform.getRotation().getY(),
                      Units.degreesToRadians(
                          photonCamera.getLatestResult().getBestTarget().getPitch()));
          }
        }
          if (photonResults.targets.size() > 1
              || photonResults.targets.get(0).getPoseAmbiguity() < APRILTAG_AMBIGUITY_THRESHOLD) {
            photonPoseEstimator
                .update(photonResults)
                .ifPresent(
                    estimatedRobotPose -> {
                      RobotPose = estimatedRobotPose.estimatedPose;
                      var estimatedPose = estimatedRobotPose.estimatedPose;
                      // Make sure the measurement is on the field
                      if (estimatedPose.getX() > 0.0
                          && estimatedPose.getX() <= FIELD_LENGTH_METERS
                          && estimatedPose.getY() > 0.0
                          && estimatedPose.getY() <= FIELD_WIDTH_METERS) {
                        atomicEstimatedRobotPose.set(estimatedRobotPose);
                        updates.add(getPoseAtTimestamp(timestamp));
                        RobotContainer.poseEstimator.addVisionData(updates);
                        updates.clear();
                      }
                    });
          }
        }
      }
    }
    Logger.recordOutput("Odometry/Photonvision", RobotPose.toPose2d());
    Logger.recordOutput("Odometry/Vision Speaker Distance", distanceToSpeaker);
  }

  private static final double speakerHeight = Units.inchesToMeters(80.5);
  private static final double ampHeight = Units.inchesToMeters(38);
  private static final double sourceHeight = Units.inchesToMeters(0);
  private static final double stageHeight = Units.inchesToMeters(0);

  private static final Pose3d speakerRed =
      new Pose3d(16.58, 5.55, speakerHeight, new Rotation3d(0, 0, Math.toRadians(180)));
  private static final Pose3d ampRed =
      new Pose3d(14.70, 8.20, ampHeight, new Rotation3d(0, 0, Math.toRadians(-90)));
  private static final Pose3d sourceRed =
      new Pose3d(0.91, 0.565, sourceHeight, new Rotation3d(0, 0, Math.toRadians(60)));
  private static final Pose3d[] nearestStageRed = {
    new Pose3d(11.90, 3.71, stageHeight, new Rotation3d(0, 0, Math.toRadians(-60))),
    new Pose3d(11.90, 4.50, stageHeight, new Rotation3d(0, 0, Math.toRadians(60))),
    new Pose3d(11.22, 4.11, stageHeight, new Rotation3d(0, 0, Math.toRadians(180)))
  };

  private static final Pose3d speakerBlue =
      new Pose3d(-0.04, 5.55, speakerHeight, new Rotation3d(0, 0, Math.toRadians(0)));
  private static final Pose3d ampBlue =
      new Pose3d(1.84, 8.20, ampHeight, new Rotation3d(0, 0, Math.toRadians(-90)));
  private static final Pose3d sourceBlue =
      new Pose3d(15.635, 0.565, sourceHeight, new Rotation3d(0, 0, Math.toRadians(120)));
  private static final Pose3d[] nearestStageBlue = {
    new Pose3d(5.32, 4.11, stageHeight, new Rotation3d(0, 0, Math.toRadians(0))),
    new Pose3d(4.64, 4.50, stageHeight, new Rotation3d(0, 0, Math.toRadians(120))),
    new Pose3d(4.64, 3.71, stageHeight, new Rotation3d(0, 0, Math.toRadians(-120)))
  };

  private static final Pose3d[] arrTranslations = {
    null,
    sourceBlue,
    sourceBlue,
    speakerRed,
    speakerRed,
    ampRed,
    ampBlue,
    speakerBlue,
    speakerBlue,
    sourceRed,
    sourceRed,
    nearestStageRed[0],
    nearestStageRed[1],
    nearestStageRed[2],
    nearestStageBlue[0],
    nearestStageBlue[1],
    nearestStageBlue[2]
  };

  public double distanceToTargetPose() {
    if (!photonResults.hasTargets()) {
      return -1;
    }
    Pose3d targetPose = arrTranslations[photonResults.getBestTarget().getFiducialId()];
    return Math.hypot(RobotPose.getX() - targetPose.getX(), RobotPose.getY() - targetPose.getY());
  }

  public Pose3d getRobotPose3d() {
    return RobotPose;
  }

  // public Pose2d getRobotPose2d() {
  //   return RobotPose.toPose2d();
  // }

  private static final Rotation3d camRotation =
      new Rotation3d(CAMERA_ROLL, CAMERA_PITCH, CAMERA_YAW);

  public Transform3d transformToTagWithoutPose() {
    if (!photonResults.hasTargets()) {
      return null;
    }

    PhotonTrackedTarget target = photonResults.getBestTarget();
    Transform3d initTrans = target.getBestCameraToTarget();
    double yaw = CAMERA_YAW;
    double pitch = CAMERA_PITCH * -1;

    // Rotating about X-Z plane
    double newX = initTrans.getX() * Math.cos(pitch) - initTrans.getZ() * Math.sin(pitch);
    double newZ = initTrans.getX() * Math.sin(pitch) + initTrans.getZ() * Math.cos(pitch);

    // Rotation about X-Y plane
    double Y = initTrans.getY();
    double newY = Y * Math.cos(yaw) - newX * Math.sin(yaw);
    newX = Y * Math.sin(yaw) + newX * Math.cos(yaw);

    Rotation3d tagRotation = initTrans.getRotation();
    Rotation3d newRotation = tagRotation.minus(camRotation);

    newX += RobotPose.getX();
    newY += RobotPose.getY();
    newZ += RobotPose.getZ();

    return new Transform3d(newX, newY, newZ, newRotation);
  }

  private static final Pose3d[] tagPoses = {
    null,
    new Pose3d(15.08, 0.25, tagHeights[1], new Rotation3d(0, 0, Math.toRadians(120))),
    new Pose3d(16.19, 0.88, tagHeights[2], new Rotation3d(0, 0, Math.toRadians(120))),
    new Pose3d(16.58, 4.98, tagHeights[3], new Rotation3d(0, 0, Math.toRadians(180))),
    new Pose3d(16.58, 5.55, tagHeights[4], new Rotation3d(0, 0, Math.toRadians(180))),
    new Pose3d(14.70, 8.20, tagHeights[5], new Rotation3d(0, 0, Math.toRadians(-90))),
    new Pose3d(1.84, 8.20, tagHeights[6], new Rotation3d(0, 0, Math.toRadians(-90))),
    new Pose3d(-0.04, 5.55, tagHeights[7], new Rotation3d(0, 0, Math.toRadians(0))),
    new Pose3d(-0.04, 4.98, tagHeights[8], new Rotation3d(0, 0, Math.toRadians(0))),
    new Pose3d(0.36, 0.88, tagHeights[9], new Rotation3d(0, 0, Math.toRadians(60))),
    new Pose3d(1.46, 0.25, tagHeights[10], new Rotation3d(0, 0, Math.toRadians(60))),
    new Pose3d(11.90, 3.71, tagHeights[11], new Rotation3d(0, 0, Math.toRadians(-60))),
    new Pose3d(11.90, 4.50, tagHeights[12], new Rotation3d(0, 0, Math.toRadians(60))),
    new Pose3d(11.22, 4.11, tagHeights[13], new Rotation3d(0, 0, Math.toRadians(180))),
    new Pose3d(5.32, 4.11, tagHeights[14], new Rotation3d(0, 0, Math.toRadians(0))),
    new Pose3d(4.64, 4.50, tagHeights[15], new Rotation3d(0, 0, Math.toRadians(120))),
    new Pose3d(4.64, 3.71, tagHeights[16], new Rotation3d(0, 0, Math.toRadians(-120)))
  };

  public Transform3d transformToTagWithPose() {
    if (!photonResults.hasTargets()) {
      return null;
    }

    return tagPoses[photonResults.getBestTarget().getFiducialId()].minus(RobotPose);
  }

  /**
   * Gets the latest robot pose. Calling this will only return the pose once. If it returns a
   * non-null value, it is a new estimate that hasn't been returned before. This pose will always be
   * for the BLUE alliance. It must be flipped if the current alliance is RED.
   *
   * @return latest estimated pose
   */
  public EstimatedRobotPose grabLatestEstimatedPose() {
    return atomicEstimatedRobotPose.getAndSet(null);
  }

  public TimestampedVisionUpdate getPoseAtTimestamp(double timestamp) {
    return new TimestampedVisionUpdate(
        timestamp,
        grabLatestEstimatedPose().estimatedPose.toPose2d(),
        VecBuilder.fill(.001, .003, 1));
  }

  /**
   * Returns the distance to speaker based on alliance (meters).
   * @return
   */
  public double getDistanceToSpeaker(){
    return distanceToSpeaker;
  }
}
