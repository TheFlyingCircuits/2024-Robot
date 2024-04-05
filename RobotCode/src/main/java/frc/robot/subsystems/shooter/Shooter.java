package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.subsystem.DiagnosticSubsystem;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends DiagnosticSubsystem {
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

        leftFlywheelsPID.setTolerance(1.0);
        rightFlywheelsPID.setTolerance(1.0);
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
     */
    public boolean flywheelsAtSetpoints() {
        return leftFlywheelsPID.atSetpoint() && rightFlywheelsPID.atSetpoint();
    }

    public void setBothFlywheelsMetersPerSecond(double metersPerSecond) {
        this.setLeftFlywheelsMetersPerSecond(metersPerSecond);
        this.setRightFlywheelsMetersPerSecond(metersPerSecond);
    }

    public Command setFlywheelSurfaceSpeedCommand(double metersPerSecond) {
        return this.run(() -> {this.setBothFlywheelsMetersPerSecond(metersPerSecond);});
    }

    public Command setFlywheelSurfaceSpeedCommand(double leftMetersPerSecond, double rightMetersPerSecond) {
        return this.run(
            () -> {
                this.setLeftFlywheelsMetersPerSecond(leftMetersPerSecond);
                this.setRightFlywheelsMetersPerSecond(rightMetersPerSecond);});
    }

    public double getWorstError() {
        double errorLeft = leftFlywheelsPID.getPositionError();
        double errorRight = rightFlywheelsPID.getPositionError();

        if (Math.abs(errorLeft) > Math.abs(errorRight)) {
            return errorLeft;
        }
        return errorRight;
    }
    

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        
        Logger.processInputs("shooterInputs", inputs);

        Logger.recordOutput("shooter/leftFlywheelsSetpoint", leftFlywheelsPID.getSetpoint());
        Logger.recordOutput("shooter/rightFlywheelsSetpoint", rightFlywheelsPID.getSetpoint());
        Logger.recordOutput("shooter/flywheelsAtSetpoints", flywheelsAtSetpoints());

    }

    @Override
    public Command autoDiagnoseCommand() {
        // TODO: Determine proper speeds and tolerances
        return Commands.sequence(
            Commands.runOnce(() -> {
                setBothFlywheelsMetersPerSecond(3);
            }, this),
            Commands.waitSeconds(1.0),
            Commands.runOnce(() -> {
                this.addFaults(io.getLeftFaults(3 / ShooterConstants.flywheelCircumferenceMeters, 1, true));
                this.addFaults(io.getRightFaults(3 / ShooterConstants.flywheelCircumferenceMeters, 1, true));
                setBothFlywheelsMetersPerSecond(-3);
            }, this),
            Commands.waitSeconds(1.0),
            Commands.runOnce(() -> {
                this.addFaults(io.getLeftFaults(-3 / ShooterConstants.flywheelCircumferenceMeters, 1, true));
                this.addFaults(io.getRightFaults(-3 / ShooterConstants.flywheelCircumferenceMeters, 1, true));

                io.setLeftMotorVolts(0.0);
                io.setRightMotorVolts(0.0);
            }, this).until(() -> getFaults().size() > 0)
            .andThen(() -> {
                io.setLeftMotorVolts(0.0);
                io.setRightMotorVolts(0.0);
            })
        );
    };
}
