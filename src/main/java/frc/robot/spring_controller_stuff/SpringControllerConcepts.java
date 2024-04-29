package frc.robot.spring_controller_stuff;

import edu.wpi.first.wpilibj.RobotController;
import frc.robot.VendorWrappers.Kraken;

/** A single place where you can read over all the concepts
 *  that are used in the SpringController class.
 *  This is here because I thought it would be easier to see
 *  how everything connects all in one place, rather than
 *  have to jump to different places in the code for the real implemention,
 *  which is split up for other organizaitonal reasons.
 */
public class SpringControllerConcepts {

    private static KinematicsTracker mechanism;
    private static KinematicsTracker setpoint;

    private static double expectedAccel = 0;
    private static double runningTotalOfMeasuredForces = 0;

    public static void learnAboutSpringController(double measuredPosition, double desiredPosition) {

        /* * * * Calculating Desired Acceleration * * * */

        // Calculate the position, velocity, and acceleration of the mechanism and the setpoint.
        mechanism.update(measuredPosition);
        setpoint.update(desiredPosition);

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
        double effectiveMass = 0; // TODO: Measure me! ("mass" for linear motion, "moment of inertia" for angular motion)
        double springConstant = 0; // TODO: Tune me!
        double dampingConstant = Math.sqrt(4 * effectiveMass * springConstant);

        double desiredRelativeAccel = ((-springConstant/effectiveMass)*relativePosition) + ((-dampingConstant/effectiveMass)*relativeVelocity);

        /* The acceleration of the mechanism relative to the setpoint
         * won't be the same as the acceleration of the mechanism relative
         * to the world/environment in cases where the setpoint itself is accelerating.
         * Therefore unless we'd like to do physics in the non-inertial frame of an
         * accelerating setpoint, we should compute the desired acceleration of the mechanism
         * relative to the world/environment (which we assume is always inertial)!
         */
        double desiredAccel = desiredRelativeAccel + setpoint.acceleration;


        /* * * * Calculating How Much Force/Torque is Required to Achieve the Desired Acceleration * * * */


        /* Using Newton's 2nd Law, we can compute how much force/torque
         * must be applied to our mechanism in or to achieve the desiredAccel
         */
        double desiredForce = desiredAccel * effectiveMass;

        /* Note that our mechanism will only have the desired motion
         * if the desiredForce happens to be the NET force on the mechanism.
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
         * (our applied torque from a motor) so that when it's applied to the mechanism
         * (along with all the forces we DON'T have control over), the net result
         * is the original desiredForce that's calculated above!
         *
         * This leaves us with the taks of estimating the external forces
         * acting on our mechanism, which we split into three categories:
         * 
         *   1: External forces that we know about ahead of time and
         *      that are relatively simple to model (e.g. gravity).
         * 
         *   2: External forces that we know about ahead of time,
         *      but are too much of a pain to model (e.g. aerodynamics).
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
         * 
         *       netForce       =     modeledForces      + unmodeledForces
         *          |                       |                     |
         * mass * measuredAccel = (mass * expectedAccel) + unmodeledForces
         * 
         * Solving for unmodeledForces yeilds:
         * unmodeledForces = mass * (measuredAccel - expectedAccel)
         */
        /* Note that while this method of measuring unmodeled forces should work in theory,
         * there are a few obstacles in the way of getting it working in practice:
         * 
         *   1: If the formula you use to model a known external force is wrong
         *      (or maybe just a poor approximation), then we might measure unmodeled
         *      forces that aren't really there. It's unclear to me if this is really a problem
         *      though, because this might just result in the bad formula effectively being
         *      corrected?
         * 
         *   2: If the time between when a force is requested and a force is applied isn't negligable,
         *      then we'd have to incorporate some kind of compensation that I'm not sure how to do
         *      off the top of my head.
         * 
         *   3: If our control loop isn't fast enough, then we may have problems assuming that the acceleration
         *      between loop iterations is constant. Exactly what speed is "fast enough" isn't super clear to me
         *      though. I'm not sure how you'd go about determining that.
         * 
         *   4: Getting an accurate measurement of a mechanism's acceleration can be difficult
         *      on account of sensor noise and latency.
         * 
         * While these issues have caused me to put the external force estimation on hold for the real implementation,
         * I've left it here in the concepts section because it could be really cool if we got it working!
         */
        double sumOfKnownExternalForces = 0; // TODO: fill in with known forces
        double exampleForceOfGravity = -effectiveMass * 9.81;
        sumOfKnownExternalForces += exampleForceOfGravity;

        /* By adding the unmodeled forces to a running total
         * (instead of just calculating them on this iteration and forgetting
         * about them on the next iteration), we effectively add the forces
         * to our model! This is useful for forces that don't go away
         * quickly, which is probably alot of them, but it seems to still
         * work pretty well for briefly applied disturbaces too.
         */
        double freshExternalForces = effectiveMass * (mechanism.acceleration - expectedAccel);
        runningTotalOfMeasuredForces += freshExternalForces;

        double forceToApply = desiredForce - (sumOfKnownExternalForces + runningTotalOfMeasuredForces);
        double torqueToApply = forceToApply; // The next section deals mostly with torque because it's talking about motors.
                                             // I've just been using force up until this point because it feels easier to
                                             // think about for the sake of concepts, but Force and Torque should
                                             // be largely interchangeable. You just have to be mindful of doing
                                             // the correct conversions for scenarios where you control a torque
                                             // but desire a force (e.g. a motor driving a rack and pinion) or vice versa.


        /* * * * Clipping the forceToApply Based on What's Actually Achieveable * * * */
        // TODO: better documentation in this section


        double mechanismRotationsPerMotorRotation = 1; // TODO: Fill me in!
        double motorRotationsPerMechanismRotation = 1./mechanismRotationsPerMotorRotation;
        double outputTorquePerInputTorque = motorRotationsPerMechanismRotation; // How much torque is exerted on the mechanism for every newton-meter that the stator exerts on the rotor.
        double inputTorquePerOutputTorque = 1./outputTorquePerInputTorque; // How much torque is exerted on the rotor for every newton-meter that's exerted on the mechanism.
        int numMotors = 1; // TODO: Fill me in!

        /* Calculate how much torque each motor must exert on its rotor
         * in order for the desired torque to ultimately be applied to the mechanism.
         * TODO: documentaiton/derivation? https://youtu.be/mhtF00LQQjI?feature=shared
         * TODO: better variable names?
         * 
         * inputRadius * inputAngle = outputRadius * outputAngle
         * outputRadius / inputRadius = inputAngle / outputAngle
         * radiusOfGear -> torqueOnGear
         * outputTorque / inputTorque = inputRev / outputRev
         */
        double torquePerMotor = (torqueToApply / numMotors) * inputTorquePerOutputTorque;

        /* Make sure we don't ask for more torque from the motor than it can provide.
         * TODO: documentation/derivation?
         * TODO: Measure rotor velocity directly instead of inferring from gear ratio?
         * TODO: Measure available voltage directly from the motor controller to account
         *       for resistive losses through the PDH?
         * TODO: fill in with the correct motor type (Neo vs Kraken).
         */
        double motorRadiansPerSecond = mechanism.velocity * motorRotationsPerMechanismRotation;
        double inducedVolts = -motorRadiansPerSecond * Kraken.kEMF; // induced volts oppose velocity
        double availableVolts = RobotController.getBatteryVoltage();

        double maxMotorTorque = Kraken.torquePerAmp * ((availableVolts + inducedVolts) / Kraken.windingResistance);
        double minMotorTorque = Kraken.torquePerAmp * ((-availableVolts + inducedVolts) / Kraken.windingResistance);
        if (torquePerMotor < minMotorTorque) {
            System.out.println("You asked for too much torque in the negative direction!");
            torquePerMotor = minMotorTorque;
        }
        if (torquePerMotor > maxMotorTorque) {
            System.out.println("You asked for too much torque in the positive direction!");
            torquePerMotor = maxMotorTorque;
        }

        // Update our desired torque to comply with the limited torquePerMotor
        torqueToApply = (torquePerMotor * numMotors) * outputTorquePerInputTorque;

        /* Calculate an expected acceleration for the mechanism
         * based on the force we're about to apply, as well as all known external forces.
         * If we're in a controllable regeime, then expectedAccel should be close to the desiredAccel.
         * If we're saturating the control effort (i.e. asking for more torque than the motor can provide),
         * then expectedAccel probably won't be close to the desiredAccel.
         */
        double expectedNetTorque = torqueToApply + sumOfKnownExternalForces + runningTotalOfMeasuredForces;
        expectedAccel = expectedNetTorque / effectiveMass;

        // return torqueToApply;
    }
    
}
