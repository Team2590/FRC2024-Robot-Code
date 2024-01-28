package frc.robot.subsystems.climb;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants;

import frc.robot.util.LoggedTunableNumber;

public class ClimbIOTalonFX implements ClimbIO {
    private final TalonFX left_motor = new TalonFX(100);
    private final TalonFX right_motor = new TalonFX(200);
    
    private final double maxPosition = Constants.CLIMBER_GEAR_RATIO;

    private final double speed = 0.05;

    // LoggedTunableNumber kP = new LoggedTunableNumber("kP", 0.0);
    // LoggedTunableNumber kD = new LoggedTunableNumber("kD", 0.0);
    // LoggedTunableNumber kFF = new LoggedTunableNumber("kFF", 0.0);

    // private TalonFXConfiguration config;

    public ClimbIOTalonFX() {
        // config.Slot0.kP = kP.get();
        // config.Slot0.kD = kD.get();
        // config.Slot0.kV = kFF.get();
        // left_motor.get`Configurator().apply(config);
        // right_motor.getConfigurator().apply(config);
        right_motor.setControl(new Follower(left_motor.getDeviceID(), false));
        left_motor.optimizeBusUtilization();
        right_motor.optimizeBusUtilization();
        left_motor.setPosition(0);
    }

    @Override
    public void updateInputs(ClimbIOInputs inputs) {
        inputs.positionRotations = left_motor.getPosition().getValueAsDouble();
    }

    public void up() {
        // left_motor.setControl(new PositionVoltage(newPosition));
        left_motor.set(speed);
        checkLimits();
    }

    public void down() {
        left_motor.set(-speed);
        checkLimits();
    }

    public void checkLimits() {
        if (left_motor.getPosition().getValueAsDouble() > maxPosition) {
            left_motor.stopMotor();
        } else if (left_motor.getPosition().getValueAsDouble() < 0) {
            left_motor.stopMotor();
        }
    }

    public void reset() {
        down();
    }

    // public void updateTunableNumbers() {
    //     if (kP.hasChanged(0) || kD.hasChanged(0) || kFF.hasChanged(0)) {
    //         config.Slot0.kP = kP.get();
    //         config.Slot0.kD = kD.get();
    //         config.Slot0.kV = kFF.get();
    //         left_motor.getConfigurator().apply(config);
    //         right_motor.getConfigurator().apply(config);
    //     }
    // }
}
