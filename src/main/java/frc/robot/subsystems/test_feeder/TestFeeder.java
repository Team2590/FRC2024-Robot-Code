package frc.robot.subsystems.test_feeder;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TestFeeder extends SubsystemBase {
  private TalonFX feeder_one = new TalonFX(0);
  private TalonFX feeder_two = new TalonFX(0);

  public void run() {
    feeder_one.set(0.15);
    feeder_two.set(0.15);
  }

  public void stop() {
    feeder_one.stopMotor();
    feeder_two.stopMotor();
  }
}
