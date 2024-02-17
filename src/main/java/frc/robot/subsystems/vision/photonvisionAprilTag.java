package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import java.io.IOException;

public final class photonvisionAprilTag {

  // CHANGE THIS TO NAME OF CAMERAS - NUM 1 PRIORITY
  private static NetworkTableInstance instance = NetworkTableInstance.getDefault();
  private static PhotonCamera leftCam;
  private static PhotonCamera rightCam;
  private static AprilTagFieldLayout aprilTagFieldLayout;

  /////////////////////////////////
  //////////  LEFT  CAM  //////////
  /////////////////////////////////

  // CONSTANTS FOR CAMERA HEIGHTS AND APRIL TAG HEIGHTS
  private static final double leftcamHeight = Units.inchesToMeters(2.25);

  // Respective offsets for each camera relative to the middle front of the robot
  private static final double leftcamFrontOffset = Units.inchesToMeters(0);
  private static final double leftcamRightOffset = Units.inchesToMeters(0);

  private PhotonPipelineResult leftResult;

  // Again, just in case lol
  // private double rangeLeft;
  // private double rangeRight;

  private PhotonTrackedTarget leftTarget;

  private Transform3d transformCamLeft;

  private static final double rollLeft = Math.toRadians(0.0);
  private static final double pitchLeft = Math.toRadians(45);
  private static final double yawLeft = Math.toRadians(0.0);
  private static final Transform3d RobotToCamLeft = new Transform3d(-1 * leftcamFrontOffset, -1 * leftcamRightOffset, -1 * leftcamHeight, new Rotation3d(rollLeft, pitchLeft, yawLeft));

  private static PhotonPoseEstimator poseEstimatorSourceSpeakerLeft;
  private static PhotonPoseEstimator poseEstimatorAmpStageLeft;

  private Pose3d prevPoseSingleLeft;
  private Pose3d prevPoseMultiLeft;

  /////////////////////////////////
  //////////  RIGHT  CAM  //////////
  /////////////////////////////////

  // CONSTANTS FOR CAMERA HEIGHTS AND APRIL TAG HEIGHTS
  private static final double rightcamHeight = Units.inchesToMeters(2.25);

  // Respective offsets for each camera relative to the middle front of the robot
  private static final double rightcamFrontOffset = Units.inchesToMeters(0);
  private static final double rightcamRightOffset = Units.inchesToMeters(0);

  private PhotonPipelineResult rightResult;

  // Again, just in case lol
  // private double rangeLeft;
  // private double rangeRight;

  private PhotonTrackedTarget rightTarget;

  private Transform3d transformCamRight;

  private static final double rollRight = Math.toRadians(0.0);
  private static final double pitchRight = Math.toRadians(45);
  private static final double yawRight = Math.toRadians(0.0);
  private static final Transform3d RobotToCamRight = new Transform3d(-1 * rightcamFrontOffset, -1 * rightcamRightOffset, -1 * rightcamHeight, new Rotation3d(rollRight, pitchRight, yawRight));

  private static PhotonPoseEstimator poseEstimatorSourceSpeakerRight;
  private static PhotonPoseEstimator poseEstimatorAmpStageRight;

  private Pose3d prevPoseSingleRight;
  private Pose3d prevPoseMultiRight;

  private final double speakerHeight = Units.inchesToMeters(80.5);
  private final double ampHeight = Units.inchesToMeters(38);
  private final double sourceHeight = Units.inchesToMeters(0);
  private final double stageHeight = Units.inchesToMeters(0);

  private final Translation3d speakerRed = new Translation3d(16.58, 5.55, speakerHeight);
  private final Translation3d ampRed = new Translation3d(14.70, 8.20, ampHeight);
  private final Translation3d sourceRed = new Translation3d(0.91, 0.565, sourceHeight);
  private final Translation3d[] nearestStageRed = {
    new Translation3d(11.90, 3.71, stageHeight),
    new Translation3d(11.90, 4.50, stageHeight),
    new Translation3d(11.22, 4.11, stageHeight)
  };

  private final Translation3d speakerBlue = new Translation3d(-0.04, 5.55, speakerHeight);
  private final Translation3d ampBlue = new Translation3d(1.84, 8.20, ampHeight);
  private final Translation3d sourceBlue = new Translation3d(15.635, 0.565, sourceHeight);
  private final Translation3d[] nearestStageBlue = {
    new Translation3d(5.32, 4.11, stageHeight),
    new Translation3d(4.64, 4.50, stageHeight),
    new Translation3d(4.64, 3.71, stageHeight)
  };

  private final Translation3d[] arrTranslations = {
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

  public static void init() {
    leftCam = new PhotonCamera(instance, "LeftCam");
    leftCam.setPipelineIndex(0);
    rightCam = new PhotonCamera(instance, "RightCam");
    rightCam.setPipelineIndex(0);

    try {
      aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
    } catch (IOException e) {
      e.printStackTrace();
    }
    poseEstimatorSourceSpeakerLeft = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, leftCam, RobotToCamLeft);
    poseEstimatorAmpStageLeft = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.LOWEST_AMBIGUITY, leftCam, RobotToCamLeft);

    poseEstimatorSourceSpeakerRight = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, rightCam, RobotToCamRight);
    poseEstimatorAmpStageRight = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.LOWEST_AMBIGUITY, rightCam, RobotToCamRight);
  }

  // UPDATING CAMERA RESULTS
  public void updateResults() {
    leftResult = leftCam.getLatestResult();
    leftTarget = leftResult.getBestTarget();
    if(hasTargetsLeft()){
      if(leftResult.getTargets().size() > 1){
        for(PhotonTrackedTarget i : leftResult.getTargets()){
          if((i.getFiducialId() == 7) || (i.getFiducialId() == 4)){
            leftTarget = i;
            break;
          }
        }
      }
    }

    rightResult = rightCam.getLatestResult();
    rightTarget = rightResult.getBestTarget();
    if(hasTargetsRight()){
      if(rightResult.getTargets().size()>1){
        for(PhotonTrackedTarget i : rightResult.getTargets()){
          if((i.getFiducialId() == 7) || (i.getFiducialId() == 4)){
            rightTarget = i;
            break;
          }
        }
      }
    }
  }

  /////////////////////////////////
  //////////  LEFT  CAM  //////////
  /////////////////////////////////

  // TRUE IF ANY TARGET IS DETECTED
  public boolean hasTargetsLeft() {
    if(leftResult == null){return false;}
    return leftResult.hasTargets();
  }

  // RETURNS FIDUCIALID OF THE DETECTED TAG
  public int getIDLeft() {
    if (!hasTargetsLeft()) {
      return -1;
    }
    return leftTarget.getFiducialId();
  }

  // AFTER CALIBRATING CAMERA W/ 3D VIEWING CAPABILITY
  public Transform3d getCamToTargetTransformLeft(){
    if(!hasTargetsLeft()){return null;}
    transformCamLeft = leftTarget.getBestCameraToTarget();
    double xi = transformCamLeft.getX();
    double zi = transformCamLeft.getZ();
    double deltaPitch = Math.atan2(zi,xi) + pitchLeft;
    double di = Math.sqrt(xi*xi + zi*zi);
    return new Transform3d(di * Math.cos(deltaPitch) - leftcamFrontOffset, transformCamLeft.getY() - leftcamRightOffset, di * Math.sin(deltaPitch) + leftcamHeight, transformCamLeft.getRotation());
  }

  // RETURNS DISTANCE FROM CAMERA TO TARGET IN METERS
  public double getDistanceLeft(){
    if(!hasTargetsLeft()){return -1.0;}
    double X = getCamToTargetTransformLeft().getX();
    double Y = getCamToTargetTransformLeft().getY();
    return Math.hypot(X, Y);
  }

  //RETURNS DELTA Y DISPLACEMENT TO THE DETECTED TAG
  public double getOffsetLeft(){
    if(hasTargetsLeft()){
      return getCamToTargetFinalTransformLeft().getY();
    }
    return 0;
  }

  // RETURNS THE ANGULAR DISPLACEMENT FROM ROBOT TO DETECTED APRILTAG
  public double getAngleLeft(){
    return Math.atan2(getCamToTargetFinalTransformLeft().getY(), getCamToTargetFinalTransformLeft().getX());
  }

  // RETURNS FINAL 3d TRANSFORMATION OF CAM TO TARGET (CONSIDERING YAW)
  public Transform3d getCamToTargetFinalTransformLeft(){
    if(!hasTargetsLeft()){return null;}
    Translation2d trans2d = PhotonUtils.estimateCameraToTargetTranslation(getDistanceLeft(), Rotation2d.fromDegrees(-leftTarget.getYaw()));
    return new Transform3d(trans2d.getX(), trans2d.getY(), getCamToTargetTransformLeft().getZ(), getCamToTargetTransformLeft().getRotation());
  }

  // USES MULTI TAG POSE ESTIMATION @ AMP & SPEAKER
  public Pose3d multiTagPose3dLeft(){
    if(!hasTargetsLeft()){return null;}
    try{
      prevPoseMultiLeft = poseEstimatorSourceSpeakerLeft.update(leftResult).get().estimatedPose;
      return prevPoseMultiLeft;
    } catch(Exception e) {
      return prevPoseMultiLeft;
    }
  }

  // USES SINGLE TAG POSE ESTIMATION @ SOURCE & STAGE
  public Pose3d singleTagPose3dLeft(){
    if(!hasTargetsLeft()){return null;}
    try{
      prevPoseSingleLeft = poseEstimatorAmpStageLeft.update(leftResult).get().estimatedPose;
      return prevPoseSingleLeft;
    } catch(Exception e) {
      return prevPoseSingleLeft;
    }
  }

  // DECIDES WHICH POSE ESTIMATION TO USE AND ESTIMATES POSE OF ROBOT
  public Pose3d getRobotPoseLeft(){
    if(!hasTargetsLeft()){return null;}
    if(leftResult.getTargets().size() > 1){
      return multiTagPose3dLeft();
    } else {
      return singleTagPose3dLeft();
    }
  }

  /////////////////////////////////
  //////////  RIGHT CAM  //////////
  /////////////////////////////////

  // TRUE IF ANY TARGET IS DETECTED
  public boolean hasTargetsRight() {
    if(rightResult == null){return false;}
    return rightResult.hasTargets();
  }

  // RETURNS FIDUCIALID OF THE DETECTED TAG
  public int getIDRight() {
    if (!hasTargetsRight()) {
      return -1;
    }
    return rightTarget.getFiducialId();
  }

  // AFTER CALIBRATING CAMERA W/ 3D VIEWING CAPABILITY
  public Transform3d getCamToTargetTransformRight(){
    if(!hasTargetsRight()){return null;}
    transformCamRight = rightTarget.getBestCameraToTarget();
    double xi = transformCamRight.getX();
    double zi = transformCamRight.getZ();
    double deltaPitch = Math.atan2(zi,xi) + pitchRight;
    double di = Math.sqrt(xi*xi + zi*zi);
    return new Transform3d(di * Math.cos(deltaPitch) - rightcamFrontOffset, transformCamRight.getY() - rightcamRightOffset, di * Math.sin(deltaPitch) + rightcamHeight, transformCamRight.getRotation());
    
  }

  // RETURNS DISTANCE FROM CAMERA TO TARGET IN METERS
  public double getDistanceRight(){
    if(!hasTargetsRight()){return -1.0;}
    double X = getCamToTargetTransformRight().getX();
    double Y = getCamToTargetTransformRight().getY();
    return Math.sqrt(X*X + Y*Y);
  }

  //RETURNS DELTA Y DISPLACEMENT TO THE DETECTED TAG
  public double getOffsetRight(){
    if(hasTargetsRight()){
      return getCamToTargetFinalTransformRight().getY();
    }
    return 0;
  }

  // RETURNS THE ANGULAR DISPLACEMENT FROM ROBOT TO DETECTED APRILTAG
  public double getAngleRight(){
    return Math.atan2(getCamToTargetFinalTransformRight().getY(),getCamToTargetFinalTransformRight().getX());
  }

  // RETURNS FINAL 3d TRANSFORMATION OF CAM TO TARGET (CONSIDERS YAW)
  public Transform3d getCamToTargetFinalTransformRight(){
    if(!hasTargetsRight()){return null;}
    Translation2d trans2d = PhotonUtils.estimateCameraToTargetTranslation(getDistanceRight(), Rotation2d.fromDegrees(-rightTarget.getYaw()));
    return new Transform3d(trans2d.getX(), trans2d.getY(), getCamToTargetTransformRight().getZ(), getCamToTargetTransformRight().getRotation());
  }

  // USES MULTI TAG POSE ESTIMATION @ AMP & SPEAKER
  public Pose3d multiTagPose3dRight(){
    if(!hasTargetsRight()){return null;}
    try{
      prevPoseMultiRight = poseEstimatorSourceSpeakerRight.update(rightResult).get().estimatedPose;
      return prevPoseMultiRight;
    } catch(Exception e) {
      return prevPoseMultiRight;
    }
  }

  // USES SINGLE TAG POSE ESTIMATION @ SOURCE & STAGE
  public Pose3d singleTagPose3dRight(){
    if(!hasTargetsRight()){return null;}
    try{
      prevPoseSingleRight = poseEstimatorAmpStageRight.update(rightResult).get().estimatedPose;
      return prevPoseSingleRight;
    } catch(Exception e) {
      return prevPoseSingleRight;
    }
  }

  // DECIDES WHICH POSE ESTIMATION TO USE AND ESTIMATES POSE OF ROBOT
  public Pose3d getRobotPoseRight(){
    if(!hasTargetsRight()){return null;}
    if(rightResult.getTargets().size() > 1){
      return multiTagPose3dRight();
    } else {
      return singleTagPose3dRight();
    }
  }

  /////////////////////////////////
  //////////  MEGA POSE  //////////
  /////////////////////////////////

  public Pose3d getRobotMegaPose(){
    Pose3d poseLeft = getRobotPoseLeft();
    Pose3d poseRight = getRobotPoseRight();
    if(!hasTargetsRight() && !hasTargetsLeft()){return null;}
    else if(!hasTargetsRight()){return poseLeft;}
    else if(!hasTargetsLeft()){return poseRight;}
    else {
      double C1 = 1 - leftTarget.getPoseAmbiguity();
      double C2 = 1 - rightTarget.getPoseAmbiguity();
      Transform3d transRight = new Transform3d(poseRight.getX(), poseRight.getY(), poseRight.getZ(), poseRight.getRotation());
      poseLeft.times(C1);
      transRight.times(C2);
      poseLeft.plus(transRight);
      poseLeft.div(C1+C2);
      return poseLeft;
    }
  }

  // getting net hypotenuse (3d) to actual field target
  public double getHypot(Translation3d trans){
    return Math.hypot(trans.getX(), trans.getY());
  }

  // getting translation3d of robot to nearest target (speaker/amp/source/stage)
  public Translation3d getTranslateToNearestTarget(){
    if(!hasTargetsLeft() && !hasTargetsRight()){return null;}
    return arrTranslations[checkID()].minus(getRobotMegaPose().getTranslation()); 
  }

  public int checkID(){
    if(!hasTargetsLeft() && !hasTargetsRight()){return -1;}
    if(!hasTargetsRight()){return getIDLeft();}
    if(!hasTargetsLeft()){return getIDRight();}
    return getIDLeft(); //could be changed to getIDRight()
  }

  // getting 3d hypotenuse/diagonal of robot to target in meters
  public double getHypotToTarget(){
    if(!hasTargetsLeft() && !hasTargetsRight()){return -1.0;}
    return getHypot(getTranslateToNearestTarget());
  }

}