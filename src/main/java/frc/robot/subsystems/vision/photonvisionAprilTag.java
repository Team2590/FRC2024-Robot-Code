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

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import java.io.IOException;

public class PhotonvisionAprilTag {

  // CONSTANTS FOR CAMERA HEIGHTS AND APRIL TAG HEIGHTS
  private static final double camHeightLeft = Units.inchesToMeters(2.25);
  private static final double camHeightRight = Units.inchesToMeters(2.13);

  // Respective offsets for each camera relative to the middle front of the robot
  private static final double leftCamFrontOffset = Units.inchesToMeters(0);
  private static final double leftCamRightOffset = Units.inchesToMeters(-6);
  private static final double rightCamFrontOffset = Units.inchesToMeters(0);
  private static final double rightCamRightOffset = Units.inchesToMeters(6);

  // CHANGE THIS TO NAME OF CAMERAS - NUM 1 PRIORITY
  private NetworkTableInstance instance = NetworkTableInstance.getDefault();
  PhotonCamera camera0;
  PhotonCamera camera1;

  private PhotonPipelineResult resultLeft;
  private PhotonPipelineResult resultRight;

  // Again, just in case lol
  // private double rangeLeft;
  // private double rangeRight;

  private PhotonTrackedTarget targetLeft;
  private PhotonTrackedTarget targetRight;

  private Transform3d transformLeftCam;
  private Transform3d transformRightCam;

  private static final double rollLeft = Math.toRadians(0.0);
  private static final double pitchLeft = Math.toRadians(45);
  private static final double yawLeft = Math.toRadians(0.0);
  private static final Transform3d leftRobotToCam = new Transform3d(-1 * leftCamRightOffset, -1 * leftCamFrontOffset, -1 * camHeightLeft, new Rotation3d(rollLeft, pitchLeft, yawLeft));

  private static final double rollRight = Math.toRadians(0.0);
  private static final double pitchRight = Math.toRadians(45);
  private static final double yawRight = Math.toRadians(0.0);
  private static final Transform3d rightRobotToCam = new Transform3d(-1 * rightCamRightOffset, -1 * rightCamFrontOffset, -1 * camHeightRight, new Rotation3d(rollRight, pitchRight, yawRight));

  private static AprilTagFieldLayout aprilTagFieldLayout;
  private PhotonPoseEstimator poseEstimatorLeftAmpSpeaker;
  private PhotonPoseEstimator poseEstimatorRightAmpSpeaker;
  private PhotonPoseEstimator poseEstimatorLeftSourceStage;
  private PhotonPoseEstimator poseEstimatorRightSourceStage;

  private Pose3d prevPoseSingleLeft;
  private Pose3d prevPoseSingleRight;
  private Pose3d prevPoseMultiLeft;
  private Pose3d prevPoseMultiRight;

  public PhotonvisionAprilTag() {
    camera0 = new PhotonCamera(instance, "LargePhotonCam");
    camera0.setPipelineIndex(0);
    camera1 = new PhotonCamera(instance, "SmallPhotonCam");
    camera1.setPipelineIndex(0);

    try {
      aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
    } catch (IOException e) {
      e.printStackTrace();
    }
    poseEstimatorLeftAmpSpeaker = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, leftRobotToCam);
    poseEstimatorRightAmpSpeaker = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, rightRobotToCam);

    poseEstimatorLeftSourceStage = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.LOWEST_AMBIGUITY, leftRobotToCam);
    poseEstimatorRightSourceStage = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.LOWEST_AMBIGUITY, rightRobotToCam);
  }

  // UPDATING CAMERA RESULTS
  public void updateResults() {
    resultLeft = camera0.getLatestResult();
    resultRight = camera1.getLatestResult();
    targetLeft = resultLeft.getBestTarget();
    targetRight = resultRight.getBestTarget();
  }

  // TRUE IF ANY TARGET IS DETECTED
  public boolean hasTargets(String leftOrRight) {
    if (leftOrRight.equals("left")) {
      return (resultLeft).hasTargets();
    } else if (leftOrRight.equals("right")) {
      return (resultRight).hasTargets();
    } else {
      return false;
    }
  }

  // RETURNS FIDUCIALID OF THE DETECTED TAG
  public int getID(String leftOrRight) {
    if (!hasTargets(leftOrRight)) {
      return -1;
    }
    if (leftOrRight.equals("left")) {
      return (resultLeft).getBestTarget().getFiducialId();
    } else if (leftOrRight.equals("right")) {
      return (resultRight).getBestTarget().getFiducialId();
    } else {
      return -1;
    }
  }

  // AFTER CALIBRATING CAMERA W/ 3D VIEWING CAPABILITY
  public Transform3d getCamToTargetTransform(String leftOrRight){
    if(!hasTargets(leftOrRight)){return null;}
    if(leftOrRight.equals("left")){
      transformLeftCam = targetLeft.getBestCameraToTarget();
      double xi = transformLeftCam.getX();
      double zi = transformLeftCam.getZ();
      double deltaPitch = Math.atan(zi/xi) + pitchLeft;
      double di = Math.sqrt(xi*xi + zi*zi);
      return new Transform3d(di * Math.cos(deltaPitch) - leftCamFrontOffset, transformLeftCam.getY() - leftCamRightOffset, di * Math.sin(deltaPitch), transformLeftCam.getRotation());
    } else if(leftOrRight.equals("right")){
      transformRightCam = targetRight.getBestCameraToTarget();
      double xi = transformRightCam.getX();
      double zi = transformRightCam.getZ();
      double deltaPitch = Math.atan(zi/xi) + pitchRight;
      double di = Math.sqrt(xi*xi + zi*zi);
      return new Transform3d(di * Math.cos(deltaPitch) - rightCamFrontOffset, transformRightCam.getY() - rightCamRightOffset, di * Math.sin(deltaPitch), transformRightCam.getRotation());
    }
    return null;
  }

  // RETURNS DISTANCE FROM CAMERA TO TARGET IN METERS
  public double getDistance(String leftOrRight){
    if(!hasTargets(leftOrRight)){return 0.0;}
    double X = getCamToTargetTransform(leftOrRight).getX();
    double Y = getCamToTargetTransform(leftOrRight).getY();
    return Math.sqrt(X*X + Y*Y);
  }

  // RETURNS 2D TRANSLATION OF CAM TO TARGET
  public Translation2d getCamToTargetTranslation2d(String leftOrRight){
    if(!hasTargets(leftOrRight)){return null;}
    if(leftOrRight.equals("left")){
      return PhotonUtils.estimateCameraToTargetTranslation(getDistance("left"), Rotation2d.fromDegrees(-targetLeft.getYaw()));
    } else if(leftOrRight.equals("right")){
      return PhotonUtils.estimateCameraToTargetTranslation(getDistance("right"), Rotation2d.fromDegrees(-targetRight.getYaw()));
    }
    return null;
  }

  // USES MULTI TAG POSE ESTIMATION @ AMP & SPEAKER
  public Pose3d multiTagPose3d(String leftOrRight){
    if(!hasTargets(leftOrRight)){return null;}
    if(leftOrRight.equals("left")){
      try{
        prevPoseMultiLeft = poseEstimatorLeftAmpSpeaker.update(resultLeft).get().estimatedPose;
        return prevPoseMultiLeft;
      } catch(Exception e) {
        return prevPoseMultiLeft;
      }
    } else if(leftOrRight.equals("right")){
      try{
        prevPoseMultiRight = poseEstimatorRightAmpSpeaker.update(resultRight).get().estimatedPose;
        return prevPoseMultiRight;
      } catch(Exception e) {
        return prevPoseMultiRight;
      }
    }
    return null;
  }

  // USES SINGLE TAG POSE ESTIMATION @ SOURCE & STAGE
  public Pose3d singleTagPose3d(String leftOrRight){
    if(!hasTargets(leftOrRight)){return null;}
    if(leftOrRight.equals("left")){
      try{
        prevPoseSingleLeft = poseEstimatorLeftSourceStage.update(resultLeft).get().estimatedPose;
        return prevPoseSingleLeft;
      } catch(Exception e) {
        return prevPoseSingleLeft;
      }
    } else if(leftOrRight.equals("right")){
      try{
        prevPoseSingleRight = poseEstimatorRightSourceStage.update(resultRight).get().estimatedPose;
        return prevPoseSingleRight;
      } catch(Exception e) {
        return prevPoseSingleRight;
      }
    }
    return null;
  }

  // DECIDES WHICH POSE ESTIMATION TO USE AND ESTIMATES POSE OF ROBOT
  public Pose3d getRobotPose(String leftOrRight){
    if(!hasTargets(leftOrRight)){return null;}
    if(leftOrRight.equals("left")){
      if(resultLeft.getTargets().size()>1){
        return multiTagPose3d(leftOrRight);
      } else {
        return singleTagPose3d(leftOrRight);
      }
    } else if(leftOrRight.equals("right")){
      if(resultRight.getTargets().size()>1){
        return multiTagPose3d(leftOrRight);
      } else {
        return singleTagPose3d(leftOrRight);
      }
    }
    return null;
  }

}
