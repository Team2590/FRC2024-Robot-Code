package frc.robot.subsystems.vision;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;

public class PhotonvisionNote {
    private NetworkTableInstance instance = NetworkTableInstance.getDefault();
    private PhotonCamera cam;
    private PhotonPipelineResult result;
    private PhotonTrackedTarget target;

    // CONSTANTS FOR CAMERA HEIGHTS AND APRIL TAG HEIGHTS
    private static final double camHeight = Units.inchesToMeters(12);

    // Respective offsets for each camera relative to the middle front of the robot
    private static final double camFrontOffset = Units.inchesToMeters(0);
    private static final double camRightOffset = Units.inchesToMeters(0);
    //private static final double roll = Math.toRadians(0.0);
    private static final double pitch = Math.toRadians(-34);
    //private static final double yaw = Math.toRadians(0.0);
    Translation2d initTrans;

    
    public PhotonvisionNote(){
        cam = new PhotonCamera(instance, "NoteCam");
        cam.setPipelineIndex(0);
    }

    public void updateResults(){
        result = cam.getLatestResult();
        target = result.getBestTarget();
    }

    public boolean hasTargets(){
        if(result==null){return false;}
        return result.hasTargets();
    }

    public double getYaw(){
        if(result==null || !hasTargets()){
            return Math.PI;
        }
        return Math.toRadians(target.getYaw());
    }

    public double getPitch(){
        if(result==null || !hasTargets()){
            return Math.PI;
        }
        return Math.abs(Math.toRadians(target.getPitch()) + pitch);
    }

    public Translation2d getRobotToNote(){
        if(result==null || !hasTargets()){
            return null;
        }
        double distance = camHeight / Math.tan(getPitch());
        initTrans = new Translation2d(distance, getYaw());
        initTrans.plus(new Translation2d(camFrontOffset,camRightOffset));
        return initTrans;
    }

}
