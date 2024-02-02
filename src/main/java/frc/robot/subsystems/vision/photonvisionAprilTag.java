package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.io.IOException;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class PhotonvisionAprilTag {

  // CONSTANTS FOR CAMERA HEIGHTS AND APRIL TAG HEIGHTS
  private static final double camHeight = Units.inchesToMeters(2.25);

  // Respective offsets for each camera relative to the middle front of the robot
  private static final double camFrontOffset = Units.inchesToMeters(0);
  private static final double camRightOffset = Units.inchesToMeters(0);

  // CHANGE THIS TO NAME OF CAMERAS - NUM 1 PRIORITY
  private NetworkTableInstance instance = NetworkTableInstance.getDefault();
  PhotonCamera cam;

  private PhotonPipelineResult result;

  // Again, just in case lol
  // private double rangeLeft;
  // private double rangeRight;

  private PhotonTrackedTarget target;

  private Transform3d transformCam;

  private static final double roll = Math.toRadians(0.0);
  private static final double pitch = Math.toRadians(45);
  private static final double yaw = Math.toRadians(0.0);
  private static final Transform3d RobotToCam =
      new Transform3d(
          -1 * camFrontOffset,
          -1 * camRightOffset,
          -1 * camHeight,
          new Rotation3d(roll, pitch, yaw));

  private static AprilTagFieldLayout aprilTagFieldLayout;
  private PhotonPoseEstimator poseEstimatorAmpSpeaker;
  private PhotonPoseEstimator poseEstimatorSourceStage;

  private Pose3d prevPoseSingleLeft;
  private Pose3d prevPoseMulti;

  public PhotonvisionAprilTag() {
    cam = new PhotonCamera(instance, "LargePhotonCam");
    cam.setPipelineIndex(0);

    try {
      aprilTagFieldLayout =
          AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
    } catch (IOException e) {
      e.printStackTrace();
    }
    poseEstimatorAmpSpeaker =
        new PhotonPoseEstimator(
            aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, RobotToCam);
    poseEstimatorSourceStage =
        new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.LOWEST_AMBIGUITY, RobotToCam);
  }

  // UPDATING CAMERA RESULTS
  public void updateResults() {
    result = cam.getLatestResult();
    target = result.getBestTarget();
    if (hasTargets()) {
      if (result.getTargets().size() > 1) {
        for (PhotonTrackedTarget i : result.getTargets()) {
          if ((i.getFiducialId() == 7) || (i.getFiducialId() == 4)) {
            target = i;
            break;
          }
        }
      }
    }
  }

  // TRUE IF ANY TARGET IS DETECTED
  public boolean hasTargets() {
    return result.hasTargets();
  }

  // RETURNS FIDUCIALID OF THE DETECTED TAG
  public int getID() {
    if (!hasTargets()) {
      return -1;
    }
    return target.getFiducialId();
  }

  // AFTER CALIBRATING CAMERA W/ 3D VIEWING CAPABILITY
  public Transform3d getCamToTargetTransform() {
    if (!hasTargets()) {
      return null;
    }
    transformCam = target.getBestCameraToTarget();
    double xi = transformCam.getX();
    double zi = transformCam.getZ();
    double deltaPitch = Math.atan(zi / xi) + pitch;
    double di = Math.sqrt(xi * xi + zi * zi);
    return new Transform3d(
        di * Math.cos(deltaPitch) - camFrontOffset,
        transformCam.getY() - camRightOffset,
        di * Math.sin(deltaPitch) + camHeight,
        transformCam.getRotation());
  }

  // RETURNS DISTANCE FROM CAMERA TO TARGET IN METERS
  public double getDistance() {
    if (!hasTargets()) {
      return -1.0;
    }
    double X = getCamToTargetTransform().getX();
    double Y = getCamToTargetTransform().getY();
    return Math.sqrt(X * X + Y * Y);
  }

  // RETURNS DELTA Y DISPLACEMENT TO THE DETECTED TAG
  public double getOffset() {
    if (hasTargets()) {
      return getCamToTargetTransform().getY();
    }
    return 0;
  }

  // RETURNS THE ANGULAR DISPLACEMENT FROM ROBOT TO DETECTED APRILTAG
  public double getAngle() {
    return Math.atan(getCamToTargetTransform().getY() / getCamToTargetTransform().getX());
  }

  // RETURNS 2D TRANSLATION OF CAM TO TARGET
  public Translation2d getCamToTargetTranslation2d() {
    if (!hasTargets()) {
      return null;
    }
    return PhotonUtils.estimateCameraToTargetTranslation(
        getDistance(), Rotation2d.fromDegrees(-target.getYaw()));
  }

  // USES MULTI TAG POSE ESTIMATION @ AMP & SPEAKER
  public Pose3d multiTagPose3d() {
    if (!hasTargets()) {
      return null;
    }
    try {
      prevPoseMulti = poseEstimatorAmpSpeaker.update(result).get().estimatedPose;
      return prevPoseMulti;
    } catch (Exception e) {
      return prevPoseMulti;
    }
  }

  // USES SINGLE TAG POSE ESTIMATION @ SOURCE & STAGE
  public Pose3d singleTagPose3d() {
    if (!hasTargets()) {
      return null;
    }
    try {
      prevPoseSingleLeft = poseEstimatorSourceStage.update(result).get().estimatedPose;
      return prevPoseSingleLeft;
    } catch (Exception e) {
      return prevPoseSingleLeft;
    }
  }

  // DECIDES WHICH POSE ESTIMATION TO USE AND ESTIMATES POSE OF ROBOT
  public Pose3d getRobotPose() {
    if (!hasTargets()) {
      return null;
    }
    if (result.getTargets().size() > 1) {
      return multiTagPose3d();
    } else {
      return singleTagPose3d();
    }
  }
}
