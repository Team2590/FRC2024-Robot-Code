package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class Climb extends SubsystemBase {
  private ClimbIO io;
  private ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();

  public Climb(ClimbIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
  }

  public void run() {
    // if (io.getRotationCount() > ClimbConstants.MAX_ROTATIONS) {
    //   io.stop();
    // } else {
    //   io.run(0.15);
    // }
    io.run(0.15);
  }

  public void stop() {
    io.stop();
  }
}
