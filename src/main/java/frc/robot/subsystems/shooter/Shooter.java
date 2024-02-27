package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

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
            ShooterConstants.kPFlywheelsVoltsSecondsPerMeter, 
            ShooterConstants.kIFlywheelsVoltsPerMeter, 
            ShooterConstants.kDFlywheelsVoltsSecondsSquaredPerMeter);

        rightFlywheelsPID = new PIDController(
            ShooterConstants.kPFlywheelsVoltsSecondsPerMeter, 
            ShooterConstants.kIFlywheelsVoltsPerMeter, 
            ShooterConstants.kDFlywheelsVoltsSecondsSquaredPerMeter);



        flywheelsFeedforward = new SimpleMotorFeedforward(
            ShooterConstants.kSFlywheelsVolts,
            ShooterConstants.kVFlywheelsVoltsSecondsPerMeter,
            ShooterConstants.kAFlywheelsVoltsSecondsSquaredPerMeter);
    }

    /**
     * Sets the surface speed of the left set of flywheels.
     * This is theoretically the speed that this side of the note will have when exiting the shooter.
     * @param metersPerSecond - Desired surface speed in meters per second.
     */
    public void setLeftFlywheelsMetersPerSecond(double metersPerSecond) {
        double feedforwardOutput = flywheelsFeedforward.calculate(metersPerSecond);
        double pidOutput = leftFlywheelsPID.calculate(inputs.leftFlywheelsMetersPerSecond, metersPerSecond);
        io.setLeftMotorVolts(feedforwardOutput + pidOutput);
    }

    /**
     * @return - The shooter's left flywheel surface speed in meters per second
     */
    public double getLeftFlywheelsMetersPerSecond() {
        return inputs.leftFlywheelsMetersPerSecond;
    }

    /**
     * Sets the surface speed of the right set of flywheels.
     * This is theoretically the speed that this side of the note will have when exiting the shooter.
     * @param metersPerSecond - Desired surface speed in meters per second.
     */
    public void setRightFlywheelsMetersPerSecond(double metersPerSecond) {
        double feedforwardOutput = flywheelsFeedforward.calculate(metersPerSecond);
        double pidOutput = rightFlywheelsPID.calculate(inputs.rightFlywheelsMetersPerSecond, metersPerSecond);
        io.setRightMotorVolts(feedforwardOutput + pidOutput);
    }


    /**
     * Returns true if both flywheels are spinning within some threshold of their target speeds.
     * @param rpmThreshold - Max distance from the setpoint for this function to still return true, in meters per second.
     */
    public boolean flywheelsAtSetpoints(double leftSetpointMetersPerSecond, double rightSetpointMetersPerSecond, double thresholdMetersPerSecond) {

        return 
            Math.abs(leftSetpointMetersPerSecond - inputs.leftFlywheelsMetersPerSecond) < thresholdMetersPerSecond
                && Math.abs(rightSetpointMetersPerSecond - inputs.rightFlywheelsMetersPerSecond) < thresholdMetersPerSecond;
    }
    

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        
        Logger.processInputs("shooterInputs", inputs);

        Logger.recordOutput("shooter/leftFlywheelsAtSetpoint", leftFlywheelsPID.atSetpoint());
        Logger.recordOutput("shooter/rightFlywheelsAtSetpoint", rightFlywheelsPID.atSetpoint());
        Logger.recordOutput("shooter/leftFlywheelsSetpoint", leftFlywheelsPID.getSetpoint());
        Logger.recordOutput("shooter/rightFlywheelsSetpoint", rightFlywheelsPID.getSetpoint());

    };
}
