package frc.robot.subsystems.elevatorarm;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.LookupTable;

/**
 * @author Vidur Janapureddy
 */
public class ArmIOTalonFX implements ArmIO {

  TalonFX arm = new TalonFX(Constants.ArmConstants.ARM, Constants.canbus);
  CANcoder armCancoder = new CANcoder(44, "Takeover");
  LoggedTunableNumber kP = new LoggedTunableNumber("Arm/kP", 16);
  LoggedTunableNumber kI = new LoggedTunableNumber("Arm/kI", 0);
  LoggedTunableNumber kD = new LoggedTunableNumber("Arm/kD", 0);
  LoggedTunableNumber kS = new LoggedTunableNumber("Arm/kS", 0);
  LoggedTunableNumber kV = new LoggedTunableNumber("Arm/kV", 0.1);
  LoggedTunableNumber kG = new LoggedTunableNumber("Arm/kG", -0.011);
  LoggedTunableNumber MotionMagicCruiseVelocity1 =
      new LoggedTunableNumber("Arm/MotionMagicCruiseVelocity", 1500);
  LoggedTunableNumber MotionMagicAcceleration1 =
      new LoggedTunableNumber("Arm/MotionMagicAcceleration", 500);
  LoggedTunableNumber MotionMagicJerk1 = new LoggedTunableNumber("Arm/MotionMagicJerk", 2000);
  LoggedTunableNumber ff = new LoggedTunableNumber("Arm/Feedforward", 0);
  Slot0Configs slot0;
  TalonFXConfiguration cfg;
  MotionMagicConfigs mm;
  MotionMagicDutyCycle mmv;
  final DutyCycleOut m_request = new DutyCycleOut(0);
  final DutyCycleOut m_request1 = new DutyCycleOut(0.1);
  final DutyCycleOut m_request2 = new DutyCycleOut(-0.1);
  DutyCycleOut mrequest3 = new DutyCycleOut(0);
  double[] distances =
      new double[] {
        0, 0, 0, 0,
        0, 0, 0, 0,
        0, 0, 0, 0,
        0, 0, 0, 0,
        0, 0, 0, 0,
        0, 0, 0, 0,
        0, 0, 0, 0,
        0, 0, 0
      };
  double[] setpoints =
      new double[] {
        0, 0, 0, 0,
        0, 0, 0, 0,
        0, 0, 0, 0,
        0, 0, 0, 0,
        0, 0, 0, 0,
        0, 0, 0, 0,
        0, 0, 0, 0,
        0, 0, 0
      };
  private LookupTable setpointcalculator = new LookupTable(distances, setpoints);
  private double ampsetpoint = -0.18;
  private double intakesetpoint = 0.168;
  private double speakerdistance = 0;
  private final StatusSignal<Double> armpos = armCancoder.getPosition();
  private final StatusSignal<Double> armabspos = armCancoder.getAbsolutePosition();

  public ArmIOTalonFX() {
    /* configurations for the arm encoder */

    mmv = new MotionMagicDutyCycle(-0.1);

    /* configurations for the arm motor */
    cfg = new TalonFXConfiguration();
    // cfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;

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
    slot0.kG = kG.get();
    slot0.kV = kV.get();
    slot0.GravityType = GravityTypeValue.Arm_Cosine;

    FeedbackConfigs fdb = cfg.Feedback;
    fdb.RotorToSensorRatio = 266.67;
    fdb.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    fdb.FeedbackRemoteSensorID = 44;
    MagnetSensorConfigs mag = new MagnetSensorConfigs();
    mag.MagnetOffset = -0.144;
    CANcoderConfiguration can = new CANcoderConfiguration();
    can.withMagnetSensor(mag);
    armCancoder.getConfigurator().apply(can);

    arm.getConfigurator().apply(cfg);
    mmv = new MotionMagicDutyCycle(0, true, 0, 0, false, false, false);

    BaseStatusSignal.setUpdateFrequencyForAll(50.0, armpos, armabspos);
  }

  public void updateInputs(ArmIOInputs inputs) {
    BaseStatusSignal.refreshAll(armabspos, armpos);
    inputs.armabspos = armabspos.getValueAsDouble();
    inputs.armpos = armpos.getValueAsDouble();
    inputs.velDegreesPerSecond = arm.getVelocity().getValueAsDouble();
    inputs.currentAmps = arm.getStatorCurrent().getValueAsDouble();
    updateTunableNumbers();
  }

  public String print() {
    return arm.getMotionMagicIsRunning().getValue().toString();
  }

  public void setmotionmagic() {
    // mmv.Slot = 0;
    // arm.setControl(mmv.withPosition(-0.165));
    arm.setControl(mmv.withPosition(setpointcalculator.getValue(speakerdistance)));
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

  public void armmanual() {
    arm.setControl(mrequest3);
  }

  public void stop() {
    arm.setControl(m_request);
  }

  public boolean atsetpoint() {
    return Math.abs(
            setpointcalculator.getValue(speakerdistance)
                - armCancoder.getAbsolutePosition().getValueAsDouble())
        <= 0.001;
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
    if (kV.hasChanged(0)) {
      slot0.kV = kV.get();
      arm.getConfigurator().apply(cfg);
    }
    if (kG.hasChanged(0)) {
      slot0.kG = kG.get();
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
    if (ff.hasChanged(0)) {
      mmv.FeedForward = ff.get();
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
