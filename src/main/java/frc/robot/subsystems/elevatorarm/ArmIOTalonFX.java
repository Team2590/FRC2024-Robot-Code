package frc.robot.subsystems.elevatorarm;

import static frc.robot.subsystems.elevatorarm.ArmConstants.Arm.ARM_GEAR_RATIO;
import static frc.robot.subsystems.elevatorarm.ArmConstants.Arm.ARM_ID;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.commons.LoggedTunableNumber;

public class ArmIOTalonFX implements ArmIO {

  TalonFX arm = new TalonFX(ARM_ID);
  // CANCoder armAzimuth = new CANCoder(ARM_AZIMUTH_ID);

  LoggedTunableNumber kP = new LoggedTunableNumber("Arm/kP", 0.1);
  LoggedTunableNumber kI = new LoggedTunableNumber("Arm/kI", 0);
  LoggedTunableNumber kD = new LoggedTunableNumber("Arm/kD", 0);
  LoggedTunableNumber kS = new LoggedTunableNumber("Arm/kS", 0);
  LoggedTunableNumber MotionMagicCruiseVelocity1 =
      new LoggedTunableNumber("Arm/MotionMagicCruiseVelocity", 1);
  LoggedTunableNumber MotionMagicAcceleration1 =
      new LoggedTunableNumber("Arm/MotionMagicAcceleration", 0.5);
  Slot0Configs slot0;
  TalonFXConfiguration cfg;
  MotionMagicConfigs mm;
  MotionMagicDutyCycle mmv;
  final DutyCycleOut m_request = new DutyCycleOut(0);

  public ArmIOTalonFX() {
    /* configurations for the arm encoder */

    mmv = new MotionMagicDutyCycle(-0.1);
    final Mechanisms m_mechanisms = new Mechanisms();

    /* configurations for the arm motor */
    cfg = new TalonFXConfiguration();

    /* Configure current limits */
    mm = cfg.MotionMagic;
    mm.MotionMagicCruiseVelocity = MotionMagicCruiseVelocity1.get();
    mm.MotionMagicAcceleration = MotionMagicAcceleration1.get();
    mm.MotionMagicJerk = 0.1;

    slot0 = cfg.Slot0;
    slot0.kP = kP.get();
    slot0.kI = kI.get();
    slot0.kD = kD.get();
    slot0.kS = kS.get();

    FeedbackConfigs fdb = cfg.Feedback;
    fdb.SensorToMechanismRatio = ARM_GEAR_RATIO;

    arm.getConfigurator().apply(cfg);
    mmv = new MotionMagicDutyCycle(0, true, 0, 0, false, false, false);
  }

  public void updateInputs(ArmIOInputs inputs) {
    BaseStatusSignal.refreshAll(arm.getMotorVoltage(), arm.getSupplyVoltage());
    updateTunableNumbers();
  }

  public String print() {
    return arm.getMotionMagicIsRunning().getValue().toString();
  }

  public void setmotionmagic() {
    // mmv.Slot = 0;
    // arm.setControl(mmv.withPosition(-0.165));
    arm.setControl(mmv.withPosition(-0.165));
  }

  public void setmotionmagic2() {

    arm.setControl(mmv.withPosition(0));
  }

  public void resetArm() {
    arm.setPosition(0);
  }

  public void stop() {
    arm.setControl(m_request);
  }

  public void updateTunableNumbers() {
    if (kP.hasChanged(0)) {
      slot0.kP = kP.get();
      arm.getConfigurator().apply(cfg);
    }

    if (kI.hasChanged(0)) {
      slot0.kI = kI.get();
      arm.getConfigurator().apply(cfg);
    }
    if (kD.hasChanged(0)) {
      slot0.kD = kD.get();
      arm.getConfigurator().apply(cfg);
    }
    if (kS.hasChanged(0)) {
      slot0.kS = kS.get();
      arm.getConfigurator().apply(cfg);
    }
    if (MotionMagicCruiseVelocity1.hasChanged(0)) {
      mm.MotionMagicCruiseVelocity = MotionMagicCruiseVelocity1.get();
      arm.getConfigurator().apply(cfg);
    }

    if (MotionMagicAcceleration1.hasChanged(0)) {
      mm.MotionMagicAcceleration = MotionMagicAcceleration1.get();
      arm.getConfigurator().apply(cfg);
    }
  }

  // private static double CANCoderSensorUnitsToDegrees(double sensorUnits) {
  //     return sensorUnits * (360.0) / 4096.0;
  // }

  // private static double degreesToCANCoderSensorUnits(double degrees) {
  //     return degrees * 4096.0 / (360.0);
  // }

  // private static double CANCoderSensorUnitsToDegreesPerSecond(double sensorUnits) {
  //     return sensorUnits * ((360.0 * 10.0)/4096.0);
  // }

  // private static double degreesPerSecondToCANCoderSensorUnits(double degrees) {
  //     return degrees * (4096.0/(360.0 * 10.0));
  // }

  // private double getAngle() {
  //     return CANCoderSensorUnitsToDegrees(arm.getSelectedSensorPosition());
  // }

}
