package frc.robot.subsystems.elevatorarm;

import com.ctre.phoenix6.controls.DutyCycleOut;
// import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.ArmConstants;

/**
 * IO class for a simulated arm subsystem
 * 
 * @author Ian Keller
 */
public class ArmIOSim implements ArmIO {
    // TODO: find the guessed value
    private final DCMotorSim armSim = new DCMotorSim(DCMotor.getFalcon500(1), ArmConstants.ARM_GEAR_RATIO, 1.0);
    // private final TalonFX arm = new TalonFX(0);
    // private final TalonFXSimState sim = arm.getSimState();

    public void updateInputs(ArmIOInputs inputs) {
        inputs.armabspos = armSim.getAngularPositionRotations();
        inputs.armpos = armSim.getAngularPositionRotations();
        // TODO: in ArmIOTalonFX, this value is in rotations per second. the sim version of this value should be verified
        inputs.velDegreesPerSecond = armSim.getAngularVelocityRPM() / 60; 
        inputs.currentAmps = Math.abs(armSim.getCurrentDrawAmps());
    }

    public void setPosition(double position) {
        armSim.setState(position * 2 * Math.PI, 0);
    }

    public void stop() {
        armSim.setInputVoltage(0);
    }

    public void setPower(DutyCycleOut power) {
        armSim.setInputVoltage(10 * Double.parseDouble(power.getControlInfo().get("Output")));
    }

}
