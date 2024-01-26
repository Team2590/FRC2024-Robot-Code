/**
 * @author Dhruv and Shashank
 */
package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
  private ArmTalonFX armMotors = new ArmTalonFX();
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

  @Override
  public void periodic() {
    armMotors.updateInputs(inputs);
    armMotors.updateInp uts(inputs);
    Logger.processInputs("Arm", inputs);
    armMotors.checkLimits();
  }

  public void testPosition() {
    System.out.println("Test position called");
    armMotors.setPosition(1);
  }
}
