package frc.robot.subsystems.vision;

import static frc.robot.Constants.FieldConstants.FIELD_LENGTH_METERS;
import static frc.robot.Constants.FieldConstants.FIELD_WIDTH_METERS;
import static frc.robot.Constants.VisionConstants.APRILTAG_AMBIGUITY_THRESHOLD;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.RobotContainer;
import frc.robot.util.AprilTag;
import frc.robot.util.PoseEstimator.TimestampedVisionUpdate;
import java.util.ArrayList;
import java.util.concurrent.atomic.AtomicReference;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
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
  private static Pose3d RobotPose = new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0));
  private static PhotonPipelineResult photonResults;
  private Transform3d cameraTransform;
  public final AprilTagFieldLayout layout;

  // for climb aligning
  private double horizontalOffset = 0;
  private int lastAprilTagId;

  public PhotonRunnable(String name, Transform3d cameraTransform3d) {
    this.photonCamera = new PhotonCamera(name);
    this.cameraTransform = cameraTransform3d;
    PhotonPoseEstimator photonPoseEstimator = null;
    layout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
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

  @Override
  public void run() {
    // Get AprilTag data
    if (photonPoseEstimator != null && photonCamera != null) {
      photonResults = photonCamera.getLatestResult();
      var timestamp = photonResults.getTimestampSeconds();
      updateHorizontalOffset();
      if (photonResults.hasTargets()) {
        Logger.recordOutput(
            "photonvision/FrontCamera/Tags In Sight", photonResults.targets.toString());
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

  public Pose3d getRobotPose3d() {
    return RobotPose;
  }

  public Pose2d getRobotPose2d() {
    return RobotPose.toPose2d();
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
        VecBuilder.fill(.001, .003, .005));
  }

  /** Updates the horizontal offset to the stage */
  private void updateHorizontalOffset() {
    if (getStageAprilTag() >= 11) {
      horizontalOffset =
          RobotContainer.poseEstimator
              .getLatestPose()
              .minus(AprilTag.getTagPose(getStageAprilTag()))
              .getY();
    }
  }

  /** Gets the horizontal offset to the stage */
  public double getHorizontalOffsetToStage() {
    return horizontalOffset;
  }

  /** Gets the current stage tag, or the last one if doesn't see one */
  public int getStageAprilTag() {
    for (PhotonTrackedTarget t : photonResults.targets) {
      if (t.getFiducialId() >= 11) {
        lastAprilTagId = t.getFiducialId();
      }
    }
    return lastAprilTagId;
  }
}
