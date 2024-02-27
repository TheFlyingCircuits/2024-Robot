package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;

public class SpinFlywheels extends Command {

    private Shooter shooter; 
    private double leftFlywheelMetersPerSecond;
    private double rightFlywheelMetersPerSecond;   

    /** Spins the shooter flywheels up to a desired surface speed, in meters per second. This does not launch the note. */
    public SpinFlywheels(double leftFlywheelMetersPerSecond, double rightFlywheelMetersPerSecond, Shooter shooter) {
        this.shooter = shooter;
        this.leftFlywheelMetersPerSecond = leftFlywheelMetersPerSecond;
        this.rightFlywheelMetersPerSecond = rightFlywheelMetersPerSecond;

        addRequirements(shooter);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        shooter.setLeftFlywheelsMetersPerSecond(leftFlywheelMetersPerSecond);
        shooter.setRightFlywheelsMetersPerSecond(rightFlywheelMetersPerSecond);
    }

    @Override
    public void end(boolean isInterrupted) {
    }

    @Override
    public boolean isFinished() {
        return shooter.flywheelsAtSetpoints(leftFlywheelMetersPerSecond, rightFlywheelMetersPerSecond, 0.5);
    }
    
}
