package frc.robot.subsystems.vision;

import static frc.robot.Constants.FieldConstants.FIELD_LENGTH_METERS;
import static frc.robot.Constants.FieldConstants.FIELD_WIDTH_METERS;
import static frc.robot.Constants.VisionConstants.APRILTAG_AMBIGUITY_THRESHOLD;
import static frc.robot.Constants.VisionConstants.RobotToCam;

import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.RobotContainer;
import frc.robot.util.PoseEstimator.TimestampedVisionUpdate;
import java.util.ArrayList;
import java.util.concurrent.atomic.AtomicReference;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;

/** Runnable that gets AprilTag data from PhotonVision. */
public class PhotonRunnable implements Runnable {

  public enum Resolution {
    RES_320_240(320, 240),
    RES_640_480(640, 480),
    RES_1280_720(1280, 720),
    RES_1280_800(1280, 800),
    RES_1920_1080(1920, 1080);

    public final int width;
    public final int height;

    private Resolution(int width, int height) {
      this.width = width;
      this.height = height;
    }
  }

  private final PhotonPoseEstimator photonPoseEstimator;
  private final PhotonCamera photonCamera;
  private final AtomicReference<EstimatedRobotPose> atomicEstimatedRobotPose =
      new AtomicReference<EstimatedRobotPose>();
  public final ArrayList<TimestampedVisionUpdate> updates =
      new ArrayList<TimestampedVisionUpdate>();
      private PhotonCameraSim m_cameraSim;


  public PhotonRunnable(String name, Transform3d transform, Resolution resolution, Rotation2d fovDiag) {
    this.photonCamera = new PhotonCamera(name);
  
    PhotonPoseEstimator photonPoseEstimator = null;
    var layout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    // PV estimates will always be blue, they'll get flipped by robot thread
    layout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
    if (photonCamera != null) {
      photonPoseEstimator =
          new PhotonPoseEstimator(
              layout,
              PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
              photonCamera,
              transform);
      photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }
    this.photonPoseEstimator = photonPoseEstimator;

    var cameraProperties = SimCameraProperties.PERFECT_90DEG();
    cameraProperties.setCalibration(resolution.width, resolution.height, fovDiag);
    this.m_cameraSim = new PhotonCameraSim(photonCamera, cameraProperties);

    // Enable wireframe in sim camera stream
    m_cameraSim.enableDrawWireframe(true);
  }

  @Override
  public void run() {
    // Get AprilTag data
    if (photonPoseEstimator != null && photonCamera != null) {
      var photonResults = photonCamera.getLatestResult();
      var timestamp = photonResults.getTimestampSeconds();

      if (!photonResults.hasTargets()) return;
      if (photonResults.targets.size() == 1
          && photonResults.targets.get(0).getPoseAmbiguity() > APRILTAG_AMBIGUITY_THRESHOLD) return;

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
                    updates.add(getPoseAtTimestamp(timestamp));
                    RobotContainer.poseEstimator.addVisionData(updates);
                    updates.clear();
                  }
                });
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

  public TimestampedVisionUpdate getPoseAtTimestamp(double timestamp) {
    return new TimestampedVisionUpdate(
        timestamp,
        grabLatestEstimatedPose().estimatedPose.toPose2d(),
        VecBuilder.fill(.001, .003, 1));
  }
}
