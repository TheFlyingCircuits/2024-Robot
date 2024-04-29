package frc.robot.spring_controller_stuff;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * TODO: documentation
 */
public class SpringController {

    private String name = null;

    public SpringController(String name) {
        this.name = name;
    }

    public double getDesiredAccel(KinematicsTracker measured, KinematicsTracker setpoint, double momentOfInertia) {
        // Get a view of the mechanism's motion relative to the setpoint
        double relativePosition = measured.position - setpoint.position;
        double relativeVelocity = measured.velocity - setpoint.velocity;

        /* A spring-mass-dashpot system obeys
         * the following differential equation:
         * a + (b/m)*v + (k/m)*x = 0
         *
         * This system is critically damped when:
         * (b/m)^2 - 4*(k/m) = 0
         * (see MIT diffy-q lecture 9 for a refresher)
         * https://youtu.be/vP-oRQqmeg4?feature=shared
         *
         * We can have our mechanism behave like a spring-mass-dashpot
         * system by imposing forces that obey the 1st equation above.
         * Furthermore, if we chose our spring constant and
         * damping constant so that they obey the 2nd equation,
         * our system will be critically damped and have no overshoot!
         */
        double springConstant = SmartDashboard.getNumber(name+"/springConstant", 0); // TODO: tune me!
        SmartDashboard.putNumber(name+"/springConstant", springConstant);
        double dampingConstant = Math.sqrt(4 * momentOfInertia * springConstant);

        double desiredRelativeAccel = ((-springConstant/momentOfInertia)*relativePosition) + ((-dampingConstant/momentOfInertia)*relativeVelocity);

        /* The acceleration of the mechanism relative to the setpoint
         * won't be the same as the acceleration of the mechanism relative
         * to the world/environment in cases where the setpoint itself is accelerating.
         * Therefore unless we'd like to do physics in the non-inertial frame of an
         * accelerating setpoint, we should compute the desired acceleration of the mechanism
         * relative to the world/environment (which we assume is always inertial)!
         */
        double desiredAccel = desiredRelativeAccel + setpoint.acceleration;

        // Log some data before returing
        String loggingPrefix = name + "/spring/";
        Logger.recordOutput(loggingPrefix+"relativePosition", relativePosition);
        Logger.recordOutput(loggingPrefix+"relativeVelocity", relativeVelocity);
        Logger.recordOutput(loggingPrefix+"desiredRelativeAccel", desiredRelativeAccel);

        Logger.recordOutput(loggingPrefix+"measuredPosition", measured.position);
        Logger.recordOutput(loggingPrefix+"measuredVelocity", measured.velocity);
        Logger.recordOutput(loggingPrefix+"measuredAccel", measured.acceleration);

        Logger.recordOutput(loggingPrefix+"setpointPosition", setpoint.position);
        Logger.recordOutput(loggingPrefix+"setpointVelocity", setpoint.velocity);
        Logger.recordOutput(loggingPrefix+"setpointAccel", setpoint.acceleration);

        Logger.recordOutput(loggingPrefix+"desiredAccel", desiredAccel);

        return desiredAccel;
    }
}
