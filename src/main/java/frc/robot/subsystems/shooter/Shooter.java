package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase implements ShooterIO {
    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged Logger = new ShooterIOInputsAutoLogged();
    private double distance;
    public double velocityRadPerSec = 0.0;
    private ShooterTalonFX motors = new ShooterTalonFX();

    public Shooter(ShooterIO io) {
        this.io = io;
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        setDistance(inputs.distance);
    }

    @Override
    public void setDistance(double distance) {
        
    }
    
    @Override
    public void periodic() {
        
    }
}
