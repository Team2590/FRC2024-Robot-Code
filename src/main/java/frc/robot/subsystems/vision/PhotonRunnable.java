package frc.robot.subsystems.vision;

import static frc.robot.Constants.FieldConstants.FIELD_LENGTH_METERS;
import static frc.robot.Constants.FieldConstants.FIELD_WIDTH_METERS;
import static frc.robot.Constants.VisionConstants.APRILTAG_AMBIGUITY_THRESHOLD;
import static frc.robot.Constants.VisionConstants.RobotToCam;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotState;
import frc.robot.RobotContainer;
import frc.robot.util.PoseEstimator.TimestampedVisionUpdate;

import java.io.IOException;
import java.util.ArrayList;
import java.util.concurrent.atomic.AtomicReference;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

/** Runnable that gets AprilTag data from PhotonVision. */
public class PhotonRunnable implements Runnable {

  private final PhotonPoseEstimator photonPoseEstimator;
  private final PhotonCamera photonCamera;
  private static final double leftcamFrontOffset = Units.inchesToMeters(0);
  private static final double leftcamRightOffset = Units.inchesToMeters(0);
  private static final double leftcamHeight=Units.inchesToMeters(2.25);
  public double targetAngle;
  /* 
   * Ayan's additons ft AarushVision
   * 
  */
  private static final double pitchLeft = Math.toRadians(45); 


  private final AtomicReference<EstimatedRobotPose> atomicEstimatedRobotPose =
      new AtomicReference<EstimatedRobotPose>();
  public final ArrayList<TimestampedVisionUpdate> updates =
      new ArrayList<TimestampedVisionUpdate>();
  private AprilTagFieldLayout aprilTagFieldLayout;

  public PhotonRunnable() {
    this.photonCamera = new PhotonCamera("SmallPhotonCamera");
    
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
              RobotToCam.inverse());



    }
    this.photonPoseEstimator = photonPoseEstimator;

    //  try {
    //   aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
    // } catch (IOException e) {
    //   e.printStackTrace();
    // }




  }

  @Override
  public void run() {
    // Get AprilTag data
    if (photonPoseEstimator != null && photonCamera != null && !RobotState.isAutonomous()) {
      var photonResults = photonCamera.getLatestResult();
      var timestamp = photonResults.getTimestampSeconds();



      if (photonResults.hasTargets()
          && (photonResults.targets.size() > 1
              || photonResults.targets.get(0).getPoseAmbiguity() < APRILTAG_AMBIGUITY_THRESHOLD)) {

        targetAngle=getAngleLeft(true, photonResults );

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
                    System.out.println("check");
                    RobotContainer.poseEstimator.addVisionData(updates); // ADD ONLY WHEN USING
                    // VISION
                    
                    /** not able to get to this point */
                    System.out.println("Should be loggoing vision pose");
                    Logger.recordOutput("VisionEstimatedPose", estimatedPose);
                    // do not clear update: updates.clear();
                  }
                });
      }
    }



  }

  public Transform3d getCamToTargetTransformLeft(boolean hasTargets,PhotonPipelineResult result){
    if(!hasTargets){return null;}
    var target = result.getBestTarget();
    var transformCamLeft=target.getBestCameraToTarget();
    double xi = transformCamLeft.getX();
    double zi = transformCamLeft.getZ();
    double deltaPitch = Math.atan(zi/xi) + pitchLeft;
    double di = Math.sqrt(xi*xi + zi*zi);
    return new Transform3d(di * Math.cos(deltaPitch) - leftcamFrontOffset, transformCamLeft.getY() - leftcamRightOffset, di * Math.sin(deltaPitch) + leftcamHeight, transformCamLeft.getRotation());
    
  }

  public double getAngleLeft(boolean hasTargets, PhotonPipelineResult result){
    return Math.atan(getCamToTargetTransformLeft(hasTargets,result ).getY()/getCamToTargetTransformLeft(hasTargets, result).getX());
  }

  /* I couldn't think of a name, have fun figuring out what this is for */ 
 



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
