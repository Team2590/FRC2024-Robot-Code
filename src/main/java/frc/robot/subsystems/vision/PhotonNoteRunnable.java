package frc.robot.subsystems.vision;

import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants.VisionConstants;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class PhotonNoteRunnable implements Runnable {
  private static NetworkTableInstance instance = NetworkTableInstance.getDefault();
  private static PhotonCamera NoteCam = new PhotonCamera(instance, "NoteCam");
  private static PhotonPipelineResult result;
  public static PhotonTrackedTarget target;
  private static double camHeight = VisionConstants.NOTE_CAMERA_HEIGHT_METERS;
  private static double camPitch = VisionConstants.NOTE_CAMERA_PITCH;
  private static double camXOffset = VisionConstants.NOTE_CAMERA_X_DISTANCE_FROM_CENTER_METERS;
  private static double camYOffset = VisionConstants.NOTE_CAMERA_Y_DISTANCE_FROM_CENTER_METERS;
  private static double noteYawRad;
  private static double camFocalLength = 0.00195; //meters

  /** Updates results on the note detection camera. */
  @Override
  public void run() {
    try {
      result = NoteCam.getLatestResult();
      if (result.hasTargets()) {
        target = result.getBestTarget();
        double mPitch = Math.toRadians(target.getPitch());
        double mYaw = Math.toRadians(target.getYaw());
        double realPitch = Math.atan2(mPitch, camFocalLength);
        double realYaw = Math.atan2(mYaw, camFocalLength);
        double distanceToNote = camHeight / (Math.tan(camPitch + realPitch));
        double xToNote = distanceToNote * Math.cos(realYaw);
        double yToNote = distanceToNote * Math.sin(realYaw);
        noteYawRad = Math.atan2(yToNote, xToNote + camXOffset);
      } else {
        target = null;
        noteYawRad = 0;
      }
    } catch (Exception e) {
      return;
    }
  }

  /**
   * Gets the yaw of target in DEGREES.
   *
   * @return yaw of target in DEGREES
   */
  public static double getYaw() {
    try {
      if (target != null && result.hasTargets()) {
        return -noteYawRad;
      } else {
        return 0;
      }
    } catch (NullPointerException e) {
      e.printStackTrace();
      return 0;
    }
  }

  // /**
  //  * Gets the pitch of target in DEGREES.
  //  *
  //  * @return pitch of target in degrees
  //  */
  // public static double getPitch() {
  //   try {
  //     if (result.hasTargets()) {
  //       return target.getPitch();
  //     } else {
  //       return 0;
  //     }
  //   } catch (NullPointerException e) {
  //     e.printStackTrace();
  //     return 0;
  //   }
  // }
}
