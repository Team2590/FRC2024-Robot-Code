package frc.robot.subsystems.drive;

import static frc.robot.Constants.VisionConstants.*;
import static frc.robot.Constants.FieldConstants.*;

import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import java.io.UncheckedIOException;
import java.util.concurrent.atomic.AtomicReference;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

/** Runnable that gets AprilTag data from PhotonVision. */
public class PhotonRunnable implements Runnable {

  private final PhotonPoseEstimator photonPoseEstimator;
  private final PhotonCamera photonCamera;
  private final AtomicReference<EstimatedRobotPose> atomicEstimatedRobotPose =
      new AtomicReference<EstimatedRobotPose>();

  public PhotonRunnable() {
    this.photonCamera = new PhotonCamera("SmallPhotonCamera");
    PhotonPoseEstimator photonPoseEstimator = null;
    try {
      var layout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
      // PV estimates will always be blue, they'll get flipped by robot thread
      layout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
      if (photonCamera != null) {
        photonPoseEstimator =
            new PhotonPoseEstimator(
                layout,
                PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                photonCamera,
                RobotToCam.inverse());
        photonPoseEstimator.setMultiTagFallbackStrategy(
            PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);
      }
    } catch (UncheckedIOException e) {
      DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
      photonPoseEstimator = null;
    }
    this.photonPoseEstimator = photonPoseEstimator;
  }

  @Override
  public void run() {
    // Get AprilTag data
    if (photonPoseEstimator != null && photonCamera != null && !RobotState.isAutonomous()) {
      var photonResults = photonCamera.getLatestResult();
      if (photonResults.hasTargets()
          && (photonResults.targets.size() > 1
              || photonResults.targets.get(0).getPoseAmbiguity() < APRILTAG_AMBIGUITY_THRESHOLD)) {
        photonPoseEstimator
            .update(photonResults)
            .ifPresent(
                estimatedRobotPose -> {
                  var estimatedPose = estimatedRobotPose.estimatedPose;
                  // Make sure the measurement is on the field
                  if (estimatedPose.getX() > 0.0
                      && estimatedPose.getX() <= FIELD_LENGTH_METERS
                      && estimatedPose.getY() > 0.0
                      && estimatedPose.getY() <= FIELD_WIDTH_METERS) {
                    atomicEstimatedRobotPose.set(estimatedRobotPose);
                  }
                });
      }
    }
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
}
