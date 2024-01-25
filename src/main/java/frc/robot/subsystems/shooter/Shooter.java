package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
    private ShooterIO io;
    private ShooterIOInputsAutoLogged inputs;

    /** PID controller for the left motor and flywheels.
     * The PID constants for this use a velocity of rotations per second
     * in order to be compatable with WPILib's PIDController class.
     */
    private PIDController leftFlywheelsPID;

    /** PID controller for the right motor and flywheels.
     * The PID constants for this use a velocity of rotations per second
     * in order to be compatable with WPILib's PIDController class.
     */
    private PIDController rightFlywheelsPID;

    /** Feedforward model for an individual set of flywheels. 
     * This assumes that the shooter's flywheel physics is 
     * symmetric on the left and right sides.
     * The feedforward constants for this use a velocity of rotations per second
     * in order to be compatable with WPILib's SimpleMotorFeedforward class.
     */
    private SimpleMotorFeedforward flywheelsFeedforward;


    public Shooter(ShooterIO io) {
        this.io = io;
        inputs = new ShooterIOInputsAutoLogged();
        
        leftFlywheelsPID = new PIDController(
            ShooterConstants.kPFlywheelsVoltsSecondsPerRotation, 
            ShooterConstants.kIFlywheelsVoltsPerRotation, 
            ShooterConstants.kDFlywheelsVoltsSecondsSquaredPerRotation);

        flywheelsFeedforward = new SimpleMotorFeedforward(
            ShooterConstants.kSFlywheelsVolts,
            ShooterConstants.kVFlywheelsVoltsSecondsPerRotation,
            ShooterConstants.kAFlywheelsVoltsSecondsSquaredPerRotation);
    }

    public void setLeftFlywheelsRotationsPerSecond(double rotationsPerSecond) {
        double feedforwardOutput = flywheelsFeedforward.calculate(rotationsPerSecond);
        double pidOutput = leftFlywheelsPID.calculate(inputs.leftFlywheelsRotationsPerSecond, rotationsPerSecond);
        io.setLeftMotorVolts(feedforwardOutput + pidOutput);
    }

    public void setRightFlywheelsRotationsPerSecond(double rotationsPerSecond) {
        double feedforwardOutput = flywheelsFeedforward.calculate(rotationsPerSecond);
        double pidOutput = rightFlywheelsPID.calculate(inputs.rightFlywheelsRotationsPerSecond, rotationsPerSecond);
        io.setRightMotorVolts(feedforwardOutput + pidOutput);
    }

    public void setLeftFlywheelsRPM(double rpm) {
        setLeftFlywheelsRotationsPerSecond(rpm*60);
    }

    public void setRightFlywheelsRPM(double rpm) {
        setRightFlywheelsRotationsPerSecond(rpm*60);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
    };
}
