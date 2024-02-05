package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.PoseEstimator.TimestampedVisionUpdate;
import java.io.IOException;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class photonvisionAprilTag extends SubsystemBase {

  // CONSTANTS FOR CAMERA HEIGHTS AND APRIL TAG HEIGHTS

  // Respective offsets for each camera relative to the middle front of the robot
  private TimestampedVisionUpdate pose2d =
      new TimestampedVisionUpdate(0.0, new Pose2d(), VecBuilder.fill(0.00005, 0.00005, 1.0E6));

  // CHANGE THIS TO NAME OF CAMERAS - NUM 1 PRIORITY
  // private NetworkTableInstance instance = NetworkTableInstance.getDefault();
  PhotonCamera cam;

  private PhotonPipelineResult result;

  // Again, just in case lol
  // private double rangeLeft;
  // private double rangeRight;

  private PhotonTrackedTarget target;

  private Transform3d transformCam;

  private static AprilTagFieldLayout aprilTagFieldLayout;
  private PhotonPoseEstimator poseEstimatorAmpSpeaker;
  private PhotonPoseEstimator poseEstimatorSourceStage;
  private final PhotonPoseEstimator odometry;

  private Pose3d prevPoseSingleLeft;
  private Pose3d prevPoseMulti = new Pose3d();

  public photonvisionAprilTag() {
    cam = new PhotonCamera("LargePhotonCam");
    cam.setPipelineIndex(0);

    try {
      aprilTagFieldLayout =
          AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
    } catch (IOException e) {
      e.printStackTrace();
    }
    poseEstimatorAmpSpeaker =
        new PhotonPoseEstimator(
            aprilTagFieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            cam,
            Constants.RobotToCam);

    odometry =
        new PhotonPoseEstimator(
            aprilTagFieldLayout,
            PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            cam,
            Constants.RobotToCam);

    odometry.setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);

    poseEstimatorSourceStage =
        new PhotonPoseEstimator(
            aprilTagFieldLayout, PoseStrategy.LOWEST_AMBIGUITY, cam, Constants.RobotToCam);
  }

  @Override
  public void periodic() {
    PhotonPipelineResult result = cam.getLatestResult();

    if (result == null || !result.hasTargets()) {
      System.err.println(" No targets or null");
      return;
    }

    Optional<EstimatedRobotPose> currentPose =
        checkValidResult(result.targets) ? odometry.update(result) : Optional.empty();
    if (currentPose.isPresent()) {
      Logger.recordOutput("Odometry/AprilTagPose", currentPose.get().estimatedPose.toPose2d());
    }

    // var timestamp = result.getTimestampSeconds();
    // Pose3d pose = new Pose3d();
    // if (result.getTargets().size() > 1) {
    //   pose = poseEstimatorAmpSpeaker.update(result).get().estimatedPose;
    // } else {
    //   pose = poseEstimatorSourceStage.update(result).get().estimatedPose;
    // }
    // pose2d =
    //     new TimestampedVisionUpdate(
    //         timestamp,
    //         pose.transformBy(Constants.RobotToCam).toPose2d(),
    //         VecBuilder.fill(0.00005, 0.00005, 1.0E6));

    // Logger.recordOutput("Odometry/AprilTagPose", pose2d.pose());
  }

  // TRUE IF ANY TARGET IS DETECTED
  public boolean hasTargets() {
    if (result == null) {
      return false;
    }
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
    double deltaPitch = Math.atan(zi / xi) + Constants.CAMERA_PITCH;
    double di = Math.sqrt(xi * xi + zi * zi);
    return new Transform3d(
        di * Math.cos(deltaPitch) - Constants.CAMERA_X_DISTANCE_FROM_CENTER_METERS,
        transformCam.getY() - Constants.CAMERA_Y_DISTANCE_FROM_CENTER_METERS,
        di * Math.sin(deltaPitch) + Constants.CAMERA_HEIGHT_METERS,
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
  public TimestampedVisionUpdate multiTagPose3d() {
    try {
      prevPoseMulti = poseEstimatorAmpSpeaker.update(result).get().estimatedPose;
      System.out.println(prevPoseMulti);
      return new TimestampedVisionUpdate(
          result.getTimestampSeconds(),
          new Pose2d(
              prevPoseMulti.getX(),
              prevPoseMulti.getY(),
              prevPoseMulti.getRotation().toRotation2d()),
          VecBuilder.fill(0.00005, 0.00005, 1.0E6));
    } catch (Exception e) {
      e.printStackTrace();
      return new TimestampedVisionUpdate(
          result.getTimestampSeconds(),
          new Pose2d(
              prevPoseMulti.getX(),
              prevPoseMulti.getY(),
              prevPoseMulti.getRotation().toRotation2d()),
          VecBuilder.fill(0.00005, 0.00005, 1.0E6));
    }
  }

  // USES SINGLE TAG POSE ESTIMATION @ SOURCE & STAGE
  public TimestampedVisionUpdate singleTagPose3d() {
    if (!hasTargets()) {
      return null;
    }
    try {
      prevPoseSingleLeft = poseEstimatorSourceStage.update(result).get().estimatedPose;
      return new TimestampedVisionUpdate(
          result.getTimestampSeconds(),
          new Pose2d(
              prevPoseSingleLeft.getX(),
              prevPoseSingleLeft.getY(),
              prevPoseSingleLeft.getRotation().toRotation2d()),
          VecBuilder.fill(0.00005, 0.00005, 1.0E6));
    } catch (Exception e) {
      return new TimestampedVisionUpdate(
          result.getTimestampSeconds(),
          new Pose2d(
              prevPoseSingleLeft.getX(),
              prevPoseSingleLeft.getY(),
              prevPoseSingleLeft.getRotation().toRotation2d()),
          VecBuilder.fill(0.00005, 0.00005, 1.0E6));
    }
  }

  // DECIDES WHICH POSE ESTIMATION TO USE AND ESTIMATES POSE OF ROBOT
  public TimestampedVisionUpdate getRobotPose() {
    if (!hasTargets()) {
      return null;
    }
    if (result.getTargets().size() > 1) {
      return multiTagPose3d();
    } else {
      return singleTagPose3d();
    }
  }

  public TimestampedVisionUpdate getPose() {
    return pose2d;
  }

  private boolean checkValidResult(List<PhotonTrackedTarget> result) {
    for (PhotonTrackedTarget target : result) {
      if (target.getFiducialId() > 16) {
        return false;
      }
    }
    return true;
  }
}