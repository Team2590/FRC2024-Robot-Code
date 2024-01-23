/**
 * @author Dhruv and Shashank
 */
package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
    public ArmTalonFX armMotors = new ArmTalonFX();

    public Arm() {

    }

    public void testPosition() {
        armMotors.setPosition(0);
    }
}
