package frc.robot.subsystems.vision;

import static frc.robot.Constants.FieldConstants.FIELD_LENGTH_METERS;
import static frc.robot.Constants.FieldConstants.FIELD_WIDTH_METERS;
import static frc.robot.Constants.VisionConstants.APRILTAG_AMBIGUITY_THRESHOLD;
import static frc.robot.Constants.VisionConstants.CAMERA_HEIGHT_METERS;
import static frc.robot.Constants.VisionConstants.CAMERA_PITCH;
import static frc.robot.Constants.VisionConstants.CAMERA_ROLL;
import static frc.robot.Constants.VisionConstants.CAMERA_X_DISTANCE_FROM_CENTER_METERS;
import static frc.robot.Constants.VisionConstants.RobotToCam;
import static frc.robot.Constants.VisionConstants.CAMERA_YAW;
import static frc.robot.Constants.VisionConstants.CAMERA_Y_DISTANCE_FROM_CENTER_METERS;

import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotState;
import frc.robot.RobotContainer;
import frc.robot.util.PoseEstimator.TimestampedVisionUpdate;
import java.util.ArrayList;
import java.util.concurrent.atomic.AtomicReference;
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
  private static double distanceToTag;
  private static Pose3d RobotPose;
  private static PhotonPipelineResult photonResults;

  public PhotonRunnable() {
    this.photonCamera = new PhotonCamera("1MegapixelCam");
    PhotonPoseEstimator photonPoseEstimator = null;
    var layout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    // PV estimates will always be blue, they'll get flipped by robot thread
    layout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
    if (photonCamera != null) {
      photonPoseEstimator =
          new PhotonPoseEstimator(
              layout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, photonCamera, RobotToCam);
    }
    this.photonPoseEstimator = photonPoseEstimator;
  }

  private static final double[] tagHeights = {
    0, 1.36, 1.36, 1.45, 1.45, 1.36, 1.36, 1.45, 1.45, 1.36, 1.36, 1.32, 1.32, 1.32, 1.32, 1.32,
    1.32
  };

  @Override
  public void run() {
    // Get AprilTag data
    if (photonPoseEstimator != null && photonCamera != null && !RobotState.isAutonomous()) {
      photonResults = photonCamera.getLatestResult();
      var timestamp = photonResults.getTimestampSeconds();
      if (photonResults.hasTargets()) {
        if (photonResults.targets.get(0).getPoseAmbiguity() < APRILTAG_AMBIGUITY_THRESHOLD) {
          distanceToTag =
              PhotonUtils.calculateDistanceToTargetMeters(
                  CAMERA_HEIGHT_METERS,
                  tagHeights[photonResults.getBestTarget().getFiducialId()],
                  CAMERA_PITCH,
                  Units.degreesToRadians(
                      photonCamera.getLatestResult().getBestTarget().getPitch()));
          System.out.println(distanceToTag);

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

  public Transform3d transformToTarget() {
    if (!photonResults.hasTargets()) {
      return null;
    }
    return arrTranslations[photonResults.getBestTarget().getFiducialId()].minus(RobotPose);
  }

  private static final Rotation3d camRotation = new Rotation3d(CAMERA_ROLL, CAMERA_PITCH, CAMERA_YAW);

  public Transform3d transformToTag(){
    if (!photonResults.hasTargets()) {
      return null;
    }

    PhotonTrackedTarget target = photonResults.getBestTarget();
    Transform3d initTrans = target.getBestCameraToTarget();
    double yaw = CAMERA_YAW;
    double pitch = CAMERA_PITCH * -1;

    //Rotating about X-Z plane
    double newX = initTrans.getX() * Math.cos(pitch) - initTrans.getY() * Math.sin(pitch);
    double newZ = initTrans.getX() * Math.sin(pitch) + initTrans.getZ() * Math.cos(pitch);
    
    //Rotation about X-Y plane
    double Y = initTrans.getY();
    double newY = Y * Math.cos(yaw) - newX * Math.sin(yaw);
    newX = Y * Math.sin(yaw) + newX * Math.cos(yaw);

    Rotation3d tagRotation = initTrans.getRotation();
    Rotation3d newRotation = tagRotation.minus(camRotation);

    newX += CAMERA_X_DISTANCE_FROM_CENTER_METERS;
    newY += CAMERA_Y_DISTANCE_FROM_CENTER_METERS;
    newZ += CAMERA_HEIGHT_METERS;

    return new Transform3d(newX, newY, newZ, newRotation);
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
}
