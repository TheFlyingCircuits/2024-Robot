package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;

public class SpinFlywheels extends Command {

    private Shooter shooter; 
    private double leftTargetFlywheelRPM;
    private double rightTargetFlywheelRPM;   

    /** Spins the shooter flywheels up to a desired RPM. This does not launch the note. */
    public SpinFlywheels(double leftFlywheelRPM, double rightFlywheelRPM, Shooter shooter) {
        this.shooter = shooter;
        this.leftTargetFlywheelRPM = leftFlywheelRPM;
        this.rightTargetFlywheelRPM = rightFlywheelRPM;

        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.setLeftFlywheelsRPM(leftTargetFlywheelRPM);
        shooter.setRightFlywheelsRPM(rightTargetFlywheelRPM);
    };

    @Override
    public void execute() {};

    @Override
    public void end(boolean isInterrupted) {
    }

    @Override
    public boolean isFinished() {
        return shooter.flywheelsAtSetpoints(20);
    }
    
}
