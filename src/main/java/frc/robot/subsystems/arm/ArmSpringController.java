package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.VendorWrappers.Neo;

public class ArmSpringController {

    // Kinematics (position, velocity, acceleration)
    KinematicsTracker mechanism;
    KinematicsTracker setpoint;

    private double momentOfInertia = 0;
    private double rawMoment = 0;
    private double mechanismRotationsPerMotorRotation = 0;

    private double sumOfKnownExternalTorques = 0;
    private double runningTotalOfMeasuredTorques = 0;
    private double expectedAccel = 0;
    private LinearFilter expectedAccelFilter = LinearFilter.movingAverage(KinematicsTracker.avgSize);

    public ArmSpringController(double momentOfInertia, double mechanismRotationsPerMotorRotation, double initialPosition) {
        this.momentOfInertia = momentOfInertia;
        this.mechanismRotationsPerMotorRotation = mechanismRotationsPerMotorRotation;
        mechanism = new KinematicsTracker(initialPosition);
        setpoint = new KinematicsTracker(initialPosition);

        this.rawMoment = this.momentOfInertia;
    }

    public void reset() {
        runningTotalOfMeasuredTorques = 0;
        expectedAccelFilter.reset();
        mechanism.reset();
        setpoint.reset();
    }

    public double getDesiredAccel(double measuredPosition, double desiredPosition) {
        // Calculate some kinematic variables for the mechanism and the setpoint
        mechanism.update(measuredPosition);
        setpoint.update(desiredPosition);
        double momentScalar = SmartDashboard.getNumber("momentScalar", 1.0);
        SmartDashboard.putNumber("momentScalar", momentScalar);
        this.momentOfInertia = this.rawMoment * momentScalar;
        Logger.recordOutput("spring/momentOfInertia", momentOfInertia);

        // Get a view of the mechanism's motion relative to the setpoint
        double relativePosition = mechanism.position - setpoint.position;
        double relativeVelocity = mechanism.velocity - setpoint.velocity;

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
        double springConstant = SmartDashboard.getNumber("springConstant", 0); // TODO: tune me!
        SmartDashboard.putNumber("springConstant", springConstant); // 0.5
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
        Logger.recordOutput("spring/relativePosition", relativePosition);
        Logger.recordOutput("spring/relativeVelocity", relativeVelocity);
        Logger.recordOutput("spring/desiredRelativeAccel", desiredRelativeAccel);

        Logger.recordOutput("spring/measuredPosition", mechanism.position);
        Logger.recordOutput("spring/measuredVelocity", mechanism.velocity);
        Logger.recordOutput("spring/measuredAccel", mechanism.acceleration);

        Logger.recordOutput("spring/setpointPosition", setpoint.position);
        Logger.recordOutput("spring/setpointVelocity", setpoint.velocity);
        Logger.recordOutput("spring/setpointAccel", setpoint.acceleration);

        Logger.recordOutput("spring/desiredAccel", desiredAccel);

        return desiredAccel;
    }

    public double getIdealTorqueToApply(double measuredPosition, double desiredPosition) {
        double desiredAccel = getDesiredAccel(measuredPosition, desiredPosition);
        
        // Using the rotational version of Newton's 2nd law,
        // we can compute how much torque must be applied to our mechanism
        // in order to achieve the desiredAccel
        double desiredTorque = desiredAccel * momentOfInertia;
        desiredTorque = 0.2;

        /* Note that our mechanism will only have the desired motion
         * if the desiredTorque happens to be the NET torque on the mechanism.
         * In physics class, this is guaranteed to be the case!
         * But in robotics, there will probably be non-negligible
         * external forces acting on the mechanism that we don't have control over.
         * However, hope is not lost!
         *
         * In particular, if we can get a decent estimate of the
         * external forces acting on the mechanism, then we can
         * account for them by adding the appropriate compensation
         * forces to our final output.
         * In other words, we can modify the force that we DO have control over
         * (our applied torque from the motor) so that when it's applied to the mechanism
         * (along with all the forces we DON'T have control over), the net result
         * is the original desiredTorque that's calculated above!
         *
         * This leaves us with the taks of estimating the external forces
         * acting on the arm, which we split into three categories:
         * 
         *   1: External forces that we know about ahead of time and
         *      that are relatively simple to model (e.g. gravity).
         * 
         *   2: External forces that we know about ahead of time,
         *      but are too much of a pain to model (e.g. friction, aerodynamics).
         *
         *   3: External forces that we don't know about ahead of time
         *      (e.g. other robots crashing into us).
         * 
         * The impact of forces in the 1st category can be computed directly,
         * whereas the forces in the 2nd and 3rd categores can be inferred 
         * by comparing our expected acceleration to our actual measured acceleration.
         * 
         * Because all forces are either modeled or unmodeled, and because
         * our expectedAccel depends only on modeled forces, any deviation 
         * of the measuredAccel from the expectedAccel must be due to unmodeled forces!
         * Note that we're assuming the modeled forces are actually modeled correctly,
         * and that it's not really a problem that the control loop isn't infinitely fast!
         * 
         *       netForce       =     modeledForces      + unmodeledForces
         *          |                       |                     |
         * mass * measuredAccel = (mass * expectedAccel) + unmodeledForces
         * 
         * Solving for unmodeledForces yeilds:
         * unmodeledForces = mass * (measuredAccel - expectedAccel)
         */
        sumOfKnownExternalTorques = 0; // TODO: fill in with known forces

        /* By adding the unmodeled forces to a running total
         * (instead of just calculating them on this iteration and forgetting
         * about them on the next iteration), we effectively add the forces
         * to our model! This is useful for forces that don't go away
         * quickly, which is probably alot of them, but it seems to still
         * work pretty well for briefly applied disturbaces too.
         */
        double freshExternalTorques = momentOfInertia * (mechanism.acceleration - expectedAccel);
        runningTotalOfMeasuredTorques += freshExternalTorques;
        runningTotalOfMeasuredTorques = 0;

        double idealTorqueToApply = desiredTorque - (sumOfKnownExternalTorques + runningTotalOfMeasuredTorques);

        // Log some data before returning
        Logger.recordOutput("spring/desiredTorque", desiredTorque);
        Logger.recordOutput("spring/sumOfKnownExternalTorques", sumOfKnownExternalTorques);
        Logger.recordOutput("spring/freshExternalTorques", freshExternalTorques);
        Logger.recordOutput("spring/runningTotalOfMeasuredTorques", runningTotalOfMeasuredTorques);
        Logger.recordOutput("spring/idealTorqueToApply", idealTorqueToApply);

        return idealTorqueToApply;
    }

    public double getRealisticTorqueToApply(double measuredPosiiton, double desiredPosition, int numMotors) {
        double idealTorqueToApply = getIdealTorqueToApply(measuredPosiiton, desiredPosition);

        /* Calculate how much torque the motor must exert on the rotor
         * in order for the desired torque to ultimately be applied to the mechanism.
         * TODO: documentaiton/derivation?
         */
        double torqueOnMechanismPerTorqueOnRotor = 1./mechanismRotationsPerMotorRotation;
        double idealTorquePerMotor = (idealTorqueToApply / numMotors) / torqueOnMechanismPerTorqueOnRotor;

        /* Make sure we don't ask for more torque from the motor than it can provide.
         * TODO: documentation/derivation?
         * TODO: Measure rotor velocity directly instead of inferring from gear ratio?
         * TODO: Measure available voltage directly from the motor controller to account
         *       for resistive losses through the PDH?
         * TODO: fill in with the correct motor type (Neo vs Kraken).
         */
        double motorRadiansPerSecond = mechanism.velocity / mechanismRotationsPerMotorRotation;
        double inducedVolts = -motorRadiansPerSecond * Neo.kEMF;
        double availableVolts = RobotController.getBatteryVoltage();

        double maxTorqueFromSingleMotor = Neo.torquePerAmp * ((availableVolts + inducedVolts) / Neo.windingResistance);
        double minTorqueFromSingleMotor = Neo.torquePerAmp * ((-availableVolts + inducedVolts) / Neo.windingResistance);
        double realisticTorquePerMotor = MathUtil.clamp(idealTorquePerMotor, minTorqueFromSingleMotor, maxTorqueFromSingleMotor);

        double maxSafeTorque = SmartDashboard.getNumber("maxSafeTorque", 0); // 10
        SmartDashboard.putNumber("maxSafeTorque", maxSafeTorque);
        realisticTorquePerMotor = MathUtil.clamp(realisticTorquePerMotor, -maxSafeTorque, maxSafeTorque);


        double realisticTorqueToApply = (realisticTorquePerMotor * numMotors) * torqueOnMechanismPerTorqueOnRotor;

        /* Calculate an expected acceleration for the mechanism
         * based on the force we're about to apply, as well as all known external forces.
         * If we're in a controllable regeime, then expectedAccel should be close to the desiredAccel.
         * If we're saturating the control effort (i.e. asking for more torque than the motor can provide),
         * then expectedAccel probably won't be close to the desiredAccel.
         */
        double expectedNetTorque = realisticTorqueToApply + sumOfKnownExternalTorques + runningTotalOfMeasuredTorques;
        double rawExpectedAccel = expectedNetTorque / momentOfInertia; // TODO: SMOOTH EXPECTED ACCEL TO PREVENT PHANTOM FORCES!!!
        expectedAccel = expectedAccelFilter.calculate(rawExpectedAccel);

        // Log some values before returning
        Logger.recordOutput("spring/idealTorquePerMotor", idealTorquePerMotor);
        Logger.recordOutput("spring/maxTorqueFromSingleMotor", maxTorqueFromSingleMotor);
        Logger.recordOutput("spring/minTorqueFromSingleMotor", minTorqueFromSingleMotor);
        Logger.recordOutput("spring/realisticTorquePerMotor", realisticTorquePerMotor);
        Logger.recordOutput("spring/expectedNetTorque", expectedNetTorque);
        Logger.recordOutput("spring/expectedAccel", expectedAccel);

        return realisticTorqueToApply;
    }

    public double getVoltsPerMotor(double measuredPosition, double desiredPosition, int numMotors) {
        double torqueToApply = getRealisticTorqueToApply(measuredPosition, desiredPosition, numMotors);
        double torquePerMotor = torqueToApply / numMotors;

        // T = kT * I
        // T = kT * (V/R)
        // T = kT * (vApplied + vInduced) / R
        // vApplied = T * (R/kT) - vInduced
        // TODO: fill in with correct motor type (Neo vs Kraken)
        double motorRadiansPerSecond = mechanism.velocity / mechanismRotationsPerMotorRotation;
        double inducedVolts = -motorRadiansPerSecond * Neo.kEMF;

        double voltsToApply = torquePerMotor * (Neo.windingResistance / Neo.torquePerAmp) - inducedVolts;

        double desiredAmps = torquePerMotor / Neo.torquePerAmp;
        voltsToApply = (desiredAmps * Neo.windingResistance) - inducedVolts;

        // Log some info before returning
        Logger.recordOutput("spring/voltsToApply", voltsToApply);
        return voltsToApply;
    }

    public double getAmpsPerMotor(double measuredPosition, double desiredPosition, int numMotors) {
        double torqueToApply = getRealisticTorqueToApply(measuredPosition, desiredPosition, numMotors);
        double torquePerMotor = torqueToApply / numMotors;
        double ampsPerMotor = torquePerMotor / Neo.torquePerAmp;
        Logger.recordOutput("spring/ampsToApply", ampsPerMotor);
        return ampsPerMotor;
    }


    private class KinematicsTracker {
        public double position;
        public double velocity;
        public double acceleration;

        private double prevPosition;
        private double prevVelocity;
        private Timer timer = new Timer();

        public static int avgSize = 3; // 8 introduces too much delay and oscillation!
        private LinearFilter positionFilter = LinearFilter.movingAverage(avgSize);
        private LinearFilter velocityFilter = LinearFilter.movingAverage(avgSize);
        private LinearFilter accelFilter = LinearFilter.movingAverage(avgSize);

        public KinematicsTracker(double initialPosition) {
            this.position = initialPosition;
            this.velocity = 0;
            this.acceleration = 0;

            this.prevPosition = initialPosition;
            this.prevVelocity = 0;
            timer.restart();
        }

        public void update(double newPosition) {
            // Save information from last iteration
            // before updating for this iteration
            this.prevPosition = this.position;
            this.prevVelocity = this.velocity;

            double deltaT = timer.get();
            timer.restart();

            double rawPosition = newPosition;
            position = rawPosition; //positionFilter.calculate(rawPosition);

            double rawVelocity = (position - prevPosition) / deltaT;
            velocity = velocityFilter.calculate(rawVelocity);

            double rawAcceleration = (velocity - prevVelocity) / deltaT;
            acceleration = accelFilter.calculate(rawAcceleration);

            // position += 0.5 * acceleration * deltaT * deltaT + velocity * deltaT;
        }

        public void reset() {
            velocityFilter.reset();
            accelFilter.reset();
        }
    }
}
