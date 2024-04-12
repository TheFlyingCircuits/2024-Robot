package frc.robot.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.ArmConstants;
import frc.robot.VendorWrappers.Neo;

public class ArmSpringController {

    private Timer timer = new Timer();

    // Arm Kinematics
    private double prevMeasuredRadians;
    private double prevMeasuredVelocity;
    
    // Setpoint Kinematics
    private double prevDesiredRadians;
    private double prevSetpointVelocity;

    private double expectedAccel;
    private double implicitlyModeledTorqueTotal = 0;

    public double getVoltsPerMotor(double measuredRadians, double desiredRadians) {
        // See how much time has passed since the last control loop.
        double deltaT = timer.get();
        timer.restart();
        if (deltaT == 0) {
            // TODO: set prev stuff?
            return 0;
        }

        // Calculate some kinematic variables for the arm
        double measuredVelocity = (measuredRadians - prevMeasuredRadians) / deltaT;

        // Likewise for the setpoint
        double setpointVelocity = (desiredRadians - prevDesiredRadians) / deltaT;

        // And finally, get a view of the arm's motion
        // relative to the setpoint.
        double relativePosition = measuredRadians - desiredRadians;
        double relativeVelocity = measuredVelocity - setpointVelocity;

        /* A spring-mass-dashpot system obeys
         * the following differential equation:
         * a + (b/m)*v + (k/m)*x = 0
         *
         * This system is critically damped when:
         * (b/m)^2 - 4*(k/m) = 0
         * (see MIT diffy-q lecture 9 for a refresher)
         *
         * We can have the arm behave like a spring-mass-dashpot
         * system by imposing forces that obey the 1st equation above.
         * Furthermore, if we chose our spring constant and
         * damping constant so that they obey the 2nd equation,
         * out system will be critically damped and have no overshoot!
         * 
         * See https://www.desmos.com/calculator/cbmogxelwq for 
         * how the physical constants of our system were determined.
         * 
         * See http://hyperphysics.phy-astr.gsu.edu/hbase/mi.html
         * for the moment of inertia approximation we're using.
         */
        double armMass = Units.lbsToKilograms(18.7);
        double armLength = ArmConstants.armLengthMeters;
        double momentOfIntertia = (1./3.) * (armMass) * (armLength * armLength);
        double springConstant = 0.00001; // TODO: tune me!
        double dampingConstant = 2*momentOfIntertia*Math.sqrt(springConstant/momentOfIntertia);

        double desiredRelativeAccel = (-(springConstant/momentOfIntertia)*relativePosition) + (-(dampingConstant/momentOfIntertia)*relativeVelocity);

        /* The acceleration of the arm relative to the setpoint won't be
         * the same as the acceleration of the arm relative to the
         * world/environment in cases where the setpoint itself is
         * accelerating. Therefore, unless we'd like to do physics
         * in the non-inertial frame of an accelerating setpoint,
         * we should compute the desired acceleration of the arm
         * relative to the world/environment
         * (which we assume is always inertial)!
         */
        double setpointAccel = (setpointVelocity - prevSetpointVelocity) / deltaT;
        double desiredAccel = desiredRelativeAccel + setpointAccel;

        /* Finally, with the desiredAccel in hand,
         * we can compute a desired torque to apply
         * to the arm using the angular version of
         * Newton's 2nd Law
         */
        double desiredTorque = desiredAccel * momentOfIntertia;


        /* Note that the arm will only have the desired motion
         * if our desiredTorque happens to be the NET torque on the arm.
         * In physics class, this is guaranteed to be the case!
         * But in robotics, there will probably be non-negligible
         * external forces acting on the arm that we don't have control over.
         * However, hope is not lost!
         *
         * In particular, if we can get a decent estimate of the
         * external forces acting on the arm, then we can
         * account for them by adding the appropriate compensation
         * forces to our final output.
         * In other words, we can modify the force that we DO have control over
         * (our applied torque from the motor) so that when it's applied to the arm
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
        double knownExternalTorqueTotal = 0;

        double g = 9.81;
        double forceGravity = -armMass * g;
        double torqueGravity = (armLength / 2.) * Math.cos(measuredRadians) * forceGravity;
        knownExternalTorqueTotal += torqueGravity;

        /* By adding the unmodeled forces to an accumulator
         * (instead of just calculating them on this iteration and forgetting
         * about them on the next iteration), we effectively add the forces
         * to our model! This is useful for forces that don't go away
         * quickly, which is probably alot of them, but it seems to still
         * work pretty well for briefly applied disturbaces too.
         */
        double measuredAccel = (measuredVelocity - prevMeasuredVelocity) / deltaT;
        double unmodeledTorqueEstimate = momentOfIntertia * (measuredAccel - expectedAccel);
        implicitlyModeledTorqueTotal += unmodeledTorqueEstimate;
        double torqueAppliedToArm = desiredTorque - (knownExternalTorqueTotal + implicitlyModeledTorqueTotal);

        /* Calculate how much torque the motor must exert on the rotor
         * in order for the desired torque to ultimately be applied to the arm.
         * TODO: documentaiton/derivation?
         */
        double desiredTorquePerMotor = (torqueAppliedToArm / 2.) / ArmConstants.armGearReduction;

        /* Make sure we don't ask for more torque from the motor than it can provide.
         * TODO: documentation/derivation?
         */
        double motorRadiansPerSecond = measuredVelocity * ArmConstants.armGearReduction;
        double inducedVolts = motorRadiansPerSecond * Neo.kEMF;
        double availableVolts = RobotController.getBatteryVoltage();
        double maxTorqueFromSingleMotor = Neo.torquePerAmp * ((availableVolts - inducedVolts) / Neo.windingResistance);
        double minTorqueFromSingleMotor = Neo.torquePerAmp * ((-availableVolts - inducedVolts) / Neo.windingResistance);
        desiredTorquePerMotor = MathUtil.clamp(desiredTorquePerMotor, minTorqueFromSingleMotor, maxTorqueFromSingleMotor);
        torqueAppliedToArm = (desiredTorquePerMotor * 2) * ArmConstants.armGearReduction;


        /* Calculate an expected acceleration for the arm
         * based on the force we're about to apply, as well as all known external forces.
         * If we're in a controllable regeime, then expectedAccel should be close to the desiredAccel.
         * If we're saturating the control effort (i.e. asking for more torque than the motor can provide),
         * then expectedAccel probably won't be close to the desiredAccel.
         */
        double expectedNetTorque = torqueAppliedToArm + knownExternalTorqueTotal + implicitlyModeledTorqueTotal;
        expectedAccel = expectedNetTorque / momentOfIntertia;

        // Save values for the next iteration
        prevMeasuredRadians = measuredRadians;
        prevDesiredRadians = desiredRadians;
        prevMeasuredVelocity = measuredVelocity;
        prevSetpointVelocity = setpointVelocity;

        // Convert the motor torque into a voltage to apply
        double voltsToApply = (desiredTorquePerMotor / Neo.torquePerAmp) * (Neo.windingResistance) + inducedVolts;
        return voltsToApply;
    }
}
