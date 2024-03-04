package frc.robot.subsystems.intake;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/**
 * IO class for a simulated intake subsystem
 * 
 * @author Ian Keller
 */
public class IntakeIOSim implements IntakeIO {
    // TODO: find the guessed values
    private final DCMotorSim intakeSim = new DCMotorSim(DCMotor.getFalcon500(1), 1.0, 1.0);
    private double appliedVolts;  

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.appliedVolts = appliedVolts;
        inputs.current = Math.abs(intakeSim.getCurrentDrawAmps());
        inputs.position = intakeSim.getAngularPositionRotations();
        inputs.velocity = intakeSim.getAngularVelocityRPM();
    }

    @Override
    public void setPower(double powerPercent) {
        intakeSim.setInputVoltage(10 * powerPercent);
    }

    @Override
    public void stop() {
        intakeSim.setInputVoltage(0);
    }
}
