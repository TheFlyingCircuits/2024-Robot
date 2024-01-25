package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    

    private ShooterIO io;
    private ShooterIOInputsAutoLogged inputs;

    private PIDController leftFlywheelsPID;
    private PIDController rightFlywheelsPID;


    public Shooter(ShooterIO io) {
        this.io = io;
        inputs = new ShooterIOInputsAutoLogged();
        
    }

    @Override
    public void periodic()
}
