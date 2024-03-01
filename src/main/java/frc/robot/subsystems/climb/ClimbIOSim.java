package frc.robot.subsystems.climb;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.ClimbConstants;

/**
 * IO class for a simulated climb subsystem
 * 
 * @author Ian Keller
 */
public class ClimbIOSim implements ClimbIO {
    private final DCMotorSim leaderSim = new DCMotorSim(DCMotor.getFalcon500(2), 1.0, 1.0);
    private final DCMotorSim followerSim = new DCMotorSim(DCMotor.getFalcon500(1), 1.0, 1.0);

    @Override
    public void updateInputs(ClimbIOInputs inputs) {
        inputs.positionRotations = leaderSim.getAngularPositionRotations();
    }

    @Override
    public void run(double speed) {
        leaderSim.setInputVoltage(10 * -speed);
        followerSim.setInputVoltage(10 * -speed);
    }

    @Override
    public void setVoltage(double voltage) {
        if (Math.abs(Math.abs(getRotationCount(leaderSim)) - Math.abs(getRotationCount(followerSim)))
                > ClimbConstants.TOLERANCE) {
            followerSim.setInputVoltage(-voltage);
            leaderSim.setInputVoltage(voltage / 2);
        } else if (Math.abs(Math.abs(getRotationCount(followerSim)) - Math.abs(getRotationCount(leaderSim)))
                > ClimbConstants.TOLERANCE) {
            leaderSim.setInputVoltage(voltage);
            followerSim.setInputVoltage(-voltage / 2);
        } else {
            leaderSim.setInputVoltage(voltage);
            followerSim.setInputVoltage(-voltage);
        }
    }

    @Override
    public void stop() {
        leaderSim.setInputVoltage(0);
        followerSim.setInputVoltage(0);
    }

    @Override
    public double getRotationCount() {
        return leaderSim.getAngularPositionRotations();
    }

    private double getRotationCount(DCMotorSim motor) {
        return motor.getAngularPositionRad();
    }

    @Override
    public void resetRotationCount() {
       leaderSim.setState(0, leaderSim.getAngularVelocityRadPerSec());
       followerSim.setState(0, followerSim.getAngularVelocityRadPerSec()); 
    }
}
