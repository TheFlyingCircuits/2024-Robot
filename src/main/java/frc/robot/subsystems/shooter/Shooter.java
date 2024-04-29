package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.spring_controller_stuff.KinematicsTracker;
import frc.robot.spring_controller_stuff.SpringController;

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

    // Testing spring control stuff
    private SpringController leftSpringController = new SpringController("leftFlywheel");
    private SpringController rightSpringController = new SpringController("rightFlywheel");
    private KinematicsTracker leftKinematics = new KinematicsTracker(0);
    private KinematicsTracker rightKinematics = new KinematicsTracker(0);
    private KinematicsTracker springSetpointKinematics = new KinematicsTracker(0);


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

    public void exertTorque(double leftNewtonMeters, double rightNewtonMeters) {
        io.exertTorque(leftNewtonMeters, rightNewtonMeters);
    }

    public void testSpringControl(boolean moveMotors) {
        double stealthWheelMass = Units.lbsToKilograms(0.23);
        double stealthWheelRadius = Units.inchesToMeters(2.0);
        double stealthWheelsPerFlywheel = 4.0;
        double flywheelsPerMotor = 2.0;
        double stealthWheelMomentOfInertia = 0.5 * stealthWheelMass * stealthWheelRadius * stealthWheelRadius;
        double flywheelMomentOfInertia = stealthWheelMomentOfInertia * stealthWheelsPerFlywheel * flywheelsPerMotor;
        // Theoretical moment for
        // 1 stealth wheel  ~= 0.00013461
        // 4 stealth wheels ~= 0.00053846
        // 8 stealth wheels ~= 0.00107691
        // emperical for 8  ~= 0.00254633
        // measured is ~2.3645x theoretcial
        // delta of ~0.00146942 kg*m*m

        // Emperical value based on exerting 1 newton-meter from the motor.
        // I fit the curve y = 0.5 * a * x^2 to the position data on desmos,
        // then solved for moment using [Torque = moment * accel]
        // (this was on the left flywheel)
        flywheelMomentOfInertia = (1./392.722);

        double desiredDegrees = 0;
        springSetpointKinematics.update(Math.toRadians(desiredDegrees));
        leftKinematics.update(inputs.leftFlywheelRadians);
        rightKinematics.update(inputs.rightFlywheelRadians);

        double leftAccel = leftSpringController.getDesiredAccel(leftKinematics, springSetpointKinematics, flywheelMomentOfInertia);
        double rightAccel = rightSpringController.getDesiredAccel(rightKinematics, springSetpointKinematics, flywheelMomentOfInertia);


        double leftTorque = leftAccel * flywheelMomentOfInertia;
        double rightTorque = rightAccel * flywheelMomentOfInertia;

        // TODO: add compensation torque to overcome stiction?
        // Current value for stiction torque is reverse engineered from
        // flywheel characterization data (volts -> amps -> torque).
        // This will likely have to be tuned
        // double torqueStiction = 0.2136;
        // leftTorque += torqueStiction * Math.signum(leftKinematics.velocity);
        // rightTorque += torqueStiction * Math.signum(rightKinematics.velocity);

        if (!moveMotors) {
            leftTorque = 0;
            rightTorque = 0;
        }
        this.exertTorque(leftTorque, rightTorque);
    }

    public Command runSpringControl(boolean moveMotors) {
        return this.run(() -> {this.testSpringControl(moveMotors);});
    }
}
