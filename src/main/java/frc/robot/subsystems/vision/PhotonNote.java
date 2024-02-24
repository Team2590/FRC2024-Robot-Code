package frc.robot.subsystems.vision;

import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants.VisionConstants;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class PhotonNote {
  private static NetworkTableInstance instance = NetworkTableInstance.getDefault();
  private static PhotonCamera NoteCam;
  private static PhotonPipelineResult result;
  private static PhotonTrackedTarget target;
  private static double camHeight = VisionConstants.NOTE_CAMERA_HEIGHT_METERS;
  private static double camPitch = VisionConstants.NOTE_CAMERA_PITCH;
  private static double camXOffset = VisionConstants.NOTE_CAMERA_X_DISTANCE_FROM_CENTER_METERS;
  private static double camYOffset = VisionConstants.NOTE_CAMERA_Y_DISTANCE_FROM_CENTER_METERS;
  private static double noteXOffset = 0;
  private static double noteYOffset = 0;

  /** Initializes photonvision on the note camera */
  public static void init() {
    NoteCam = new PhotonCamera(instance, "NoteCam");
    result = NoteCam.getLatestResult();
    target = result.getBestTarget();
  }

  /** Updates results on the note detection camera. */
  public static void run() {
    result = NoteCam.getLatestResult();

    if (result.hasTargets()) {
      target = result.getBestTarget();
      noteXOffset = camHeight / (Math.tan(camPitch - Math.toRadians(target.getPitch())));
      noteYOffset = noteXOffset * Math.tan(Math.toRadians(target.getYaw())) + camYOffset;
    } else {
      target = null;
      noteXOffset = 0;
      noteYOffset = 0;
    }
  }

  /**
   * Gets the HORIZONTAL (right is +) translation from the center of the robot to the detected note.
   *
   * @return y offset of note
   */
  public static double getYOffset() {
    return noteYOffset;
  }

  public static double getXOffset() {
    return noteXOffset + camXOffset;
  }

  /**
   * Gets the yaw of target in DEGREES.
   *
   * @return yaw of target in degrees
   */
  public static double getYaw() {
    if (result.hasTargets()) {
      return target.getYaw();
    } else {
      return 0;
    }
  }

  /**
   * Gets the pitch of target in DEGREES.
   *
   * @return pitch of target in degrees
   */
  public static double getPitch() {
    if (result.hasTargets()) {
      return target.getPitch();
    } else {
      return 0;
    }
  }
}
