package frc.robot.subsystems.elevatorarm;

import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.commons.LoggedTunableNumber;


public class ArmIOTalonFX implements ArmIO {
  private double manualPower;

  TalonFX arm = new TalonFX(4);
  LoggedTunableNumber kP = new LoggedTunableNumber("Arm/kP", 2.7);
  LoggedTunableNumber kI = new LoggedTunableNumber("Arm/kI", 0);
  LoggedTunableNumber kD = new LoggedTunableNumber("Arm/kD", 1);
  LoggedTunableNumber kS = new LoggedTunableNumber("Arm/kS", 0.5);
  LoggedTunableNumber MotionMagicCruiseVelocity1 =
      new LoggedTunableNumber("Arm/MotionMagicCruiseVelocity", 1000);
  LoggedTunableNumber MotionMagicAcceleration1 =
      new LoggedTunableNumber("Arm/MotionMagicAcceleration", 500);
  LoggedTunableNumber MotionMagicJerk1 =
      new LoggedTunableNumber("Arm/MotionMagicJerk", 250);
  Slot0Configs slot0;
  TalonFXConfiguration cfg;
  MotionMagicConfigs mm;
  MotionMagicDutyCycle mmv;
  final DutyCycleOut m_request = new DutyCycleOut(0);
  final DutyCycleOut m_request1 = new DutyCycleOut(0.1);
  final DutyCycleOut m_request2 = new DutyCycleOut(-0.1);
  DutyCycleOut mrequest3 = new DutyCycleOut(0);
  private double setpoint = -0.165;
  private double ampsetpoint = -0.2;
  private double intakesetpoint = -0.35;

  public ArmIOTalonFX() {
    /* configurations for the arm encoder */

    mmv = new MotionMagicDutyCycle(-0.1);

    /* configurations for the arm motor */
    cfg = new TalonFXConfiguration();

    /* Configure current limits */
    mm = cfg.MotionMagic;
    mm.MotionMagicCruiseVelocity = MotionMagicCruiseVelocity1.get();
    mm.MotionMagicAcceleration = MotionMagicAcceleration1.get();
    mm.MotionMagicJerk = MotionMagicJerk1.get();

    slot0 = cfg.Slot0;
    slot0.kP = kP.get();
    slot0.kI = kI.get();
    slot0.kD = kD.get();
    slot0.kS = kS.get();

    FeedbackConfigs fdb = cfg.Feedback;
    fdb.SensorToMechanismRatio = 200;

    arm.getConfigurator().apply(cfg);
    mmv = new MotionMagicDutyCycle(0, true, 0, 0, false, false, false);
  }

  // public void updateInputs(ArmIOInputs inputs) {
  //   BaseStatusSignal.refreshAll(arm.getMotorVoltage(), arm.getSupplyVoltage());
  //   updateTunableNumbers();
  // }

  public String print() {
    return arm.getMotionMagicIsRunning().getValue().toString();
  }

  public void setmotionmagic() {
    // mmv.Slot = 0;
    // arm.setControl(mmv.withPosition(-0.165));
    arm.setControl(mmv.withPosition(setpoint));
  }

  // public void setmotionmagicstow() {

  //   arm.setControl(mmv.withPosition(0));
  // }
  public void setmotionmagicamp() {

    arm.setControl(mmv.withPosition(ampsetpoint));
  }
  public void setmotionmagicintake() {

    arm.setControl(mmv.withPosition(intakesetpoint));
  }

  public void resetArm() {
    arm.setPosition(0);
  }

  public void armmanualup() {
    mrequest3 = m_request1;

  }

  public void armmanualdown() {
    mrequest3 = m_request2;

  }
  public void armmanual(){
    arm.setControl(mrequest3);
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
    if (MotionMagicJerk1.hasChanged(0)) {
      mm.MotionMagicJerk = MotionMagicJerk1.get();
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
