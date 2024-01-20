package frc.robot.subsystems.vision;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CameraServerJNI;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import edu.wpi.first.wpilibj.TimedRobot;

package frc.robot.subsystems.vision;

public class photonvisionAprilTag {

    // Constants such as camera and target height stored. Change per robot and goal!'
    final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(0);
    final double TARGET_HEIGHT_METERS = Units.feetToMeters(0);
    final double HEIGHT_DIFF = Math.abs(TARGET_HEIGHT_METERS - CAMERA_HEIGHT_METERS);
    // Angle between horizontal and the camera.
    final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(0);
    final double DISTANCE_BETWEEN_CAMS = Units.feetToMeters(2);

    // How far from the target we want to be
    final double GOAL_RANGE_METERS = Units.feetToMeters(3);

    // CHANGE THIS TO NAME OF CAMERAS - NUM 1 PRIORITY
    private NetworkTableInstance instance = NetworkTableInstance.getDefault();
    PhotonCamera camera0;
    PhotonCamera camera1;

    // PID constants should be tuned per robot
    final double LINEAR_P = 0.1;
    final double LINEAR_D = 0.0;
    PIDController forwardController = new PIDController(LINEAR_P, 0, LINEAR_D);

    final double ANGULAR_P = 0.1;
    final double ANGULAR_D = 0.0;
    PIDController turnController = new PIDController(ANGULAR_P, 0, ANGULAR_D);

    private double forwardSpeedLeft;
    private double rotationSpeedLeft;
    private double forwardSpeedRight;
    private double rotationSpeedRight;

    private var resultLeft;
    private var resultRight;

    private double rangeLeft;
    private double rangeRight;
    
    public photonvisionAprilTag(){
        camera0 = new PhotonCamera(instance, "LargePhotonCam");
        camera1 = new PhotonCamera(instance, "SmallPhotonCam");
    }

    public void updateResults(){
        resultLeft = camera0.getLatestResult();
        resultRight = camera1.getLatestResult();
    }

    public boolean hasTargets(String leftOrRight){
        updateResults();
        if(leftOrRight.equals("left")){
            return resultLeft.hasTargets();
        } else if(leftOrRight.equals("right")){
            return resultRight.hasTargets();
        } else {
            return null;
        }
    }

    public int getID(String leftOrRight){
        updateResults();
        if(!hasTargets(leftOrRight)){return -1;}
        if(leftOrRight.equals("left")){
            return resultLeft.getBestTarget().getFiducialId();
        } else if(leftOrRight.equals("right")){
            return resultRight.getBestTarget().getFiducialId();
        } else {
            return -1;
        }
    }

    public double getRangeTag(String leftOrRight){
        updateResults();
        if(!hasTargets(leftOrRight)){return 0.0;}
        if(leftOrRight.equals("left")){
            rangeLeft = PhotonUtils.calculateDistanceToTargetMeters(
                CAMERA_HEIGHT_METERS,
                TARGET_HEIGHT_METERS,
                CAMERA_PITCH_RADIANS,
                resultLeft.getBestTarget().getPitch());
            return rangeLeft;
        } else if(leftOrRight.equals("right")){
            rangeRight = PhotonUtils.calculateDistanceToTargetMeters(
                CAMERA_HEIGHT_METERS,
                TARGET_HEIGHT_METERS,
                CAMERA_PITCH_RADIANS,
                resultRight.getBestTarget().getPitch());
            return rangeRight;
        } else {
            return 0.0;
        }
    }

    public double getForwardSpeed(String leftOrRight){
        updateResults();
        if(!hasTargets(leftOrRight)){return 0.0;}
        if(leftOrRight.equals("left")){
            forwardSpeedLeft = forwardController.calculate(rangeLeft, GOAL_RANGE_METERS);
            return forwardSpeedLeft;
        } else if(leftOrRight.equals("right")){
            forwardSpeedRight = forwardController.calculate(rangeLeft, GOAL_RANGE_METERS);
            return forwardSpeedRight;
        } else {
            return 0.0;
        }
    }

    public Transform3d camToTarget(String leftOrRight){
        updateResults();
        if(!hasTargets(leftOrRight)){return null;}
        if(leftOrRight.equals("left")){
            targetLeft.getBestCameraToTarget();
        } else if(leftOrRight.equals("right")){
            targetRight.getBestCameraToTarget();
        } else {
            return null;
        }
    }
}