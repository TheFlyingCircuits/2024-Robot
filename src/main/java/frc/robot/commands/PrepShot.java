package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.FieldElement;
import frc.robot.FlyingCircuitUtils;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.leds.LEDs;
import frc.robot.subsystems.shooter.Shooter;

/** Aims to shoot a note at a target by putting the
 *  arm at the correct angle, spinning the flywheels up to
 *  the right speed, and turning the drivetrain in the right direciton.
 * 
 *  The drivetrain can optionally be excluded as a requirement for instances
 *  where you only want this command to the aim the arm and spin the flywheels
 *  (mainly useful for auto where the drivetrain is being controlled by PathPlanner).
 * 
 *  In situations where you want this command to control the drivetrain, you can also
 *  inject a funciton that provides translational speed commands so that you can still
 *  move while aiming at a target. This makes the robot feel more responsive to the driver,
 *  and will come in handy when we get to implementing shoot on the move.
 * 
 *  This command will never end on its own, because you should continue aiming
 *  at a target until the note has completely exited the shooter. The intended
 *  way to use this command is to run it in parallel with a WaitUntil(prepShot::readyToShoot)
 *  command, that's followed by a fireNote() command.
*/
public class PrepShot extends Command {
    
    private Arm arm;
    private Shooter flywheels;
    private Drivetrain drivetrain;
    private Supplier<ChassisSpeeds> translationController;
    private FieldElement target;
    private Command ledFeedbackCommand;

    private boolean useAutoRotate;
    public boolean setpointsAreFresh = false;

    /** Command to aim all parts of the robot at the speaker in preparation for a shot.
     *  This command never finishes; instead use the readyToShoot() method to determine when
     *  a shot should be fired. This command also provides the ability to control translation
     *  as the robot aims.
     * 
     *  @param useAutoRotate - True if the drivetrain is able to be controlled by this command. Set this to false when prepping a shot on the move in auto.
     *  This means that the drivetrain will not move at all, only the arm and flywheels. 
     *  @param translationController - Supplier that provides chassisSpeeds so that the robot can be controlled while aiming. If useAutoRotate is false, this value is not used.
     */
    public PrepShot(boolean useAutoRotate, Drivetrain drivetrain, Arm arm, Shooter flywheels, Supplier<ChassisSpeeds> translationController, LEDs leds, FieldElement target) {
        this.drivetrain = drivetrain;
        this.arm = arm;
        this.flywheels = flywheels;
        this.translationController = translationController;
        this.useAutoRotate = useAutoRotate;
        this.target = target;

        if (useAutoRotate) {
            super.addRequirements(drivetrain, arm, flywheels);
        }
        else {
            super.addRequirements(arm, flywheels);
        }


        //       This will prob be part of the LED re-write to use progress instead of error?
        ledFeedbackCommand = leds.playAimingAnimationCommand(arm::getErrorDegrees, flywheels::getWorstError, drivetrain::getAngleError);
    }

    @Override
    public void initialize() {
        // drive angle error may be stale from last call,
        // and therefore display incorrectly on LEDs on first frame?
        ledFeedbackCommand.schedule();
        setpointsAreFresh = false;
    }

    @Override
    public void execute() {
        // Drivetrain
        if (useAutoRotate) {
            ChassisSpeeds desiredTranslationalSpeeds = translationController.get();
            Rotation2d desiredAngle = FlyingCircuitUtils.getAngleToFieldElement(target, drivetrain.getPoseMeters());
            drivetrain.fieldOrientedDriveWhileAiming(desiredTranslationalSpeeds, desiredAngle);
        }

        // Flywheels
        double leftFlywheelMetersPerSecond = 25;
        double rightFlywheelMetersPerSecond = 20;
        if (target == FieldElement.LOB_TARGET) {
            leftFlywheelMetersPerSecond = 15; //13 and 9 was too low
            rightFlywheelMetersPerSecond = 10;
        }
        flywheels.setLeftFlywheelsMetersPerSecond(leftFlywheelMetersPerSecond);
        flywheels.setRightFlywheelsMetersPerSecond(rightFlywheelMetersPerSecond);

        // Arm
        double horizontalDistance = armDistToTargetMeters();
        double verticalDistance = getTargetHeight(target) - FieldConstants.pivotHeightMeters;
        double estimatedExitVelocity = (leftFlywheelMetersPerSecond + rightFlywheelMetersPerSecond) / 2.0;

        double desiredArmAngle = arm.getDegrees();
        if (target == FieldElement.SPEAKER) {
            desiredArmAngle = getGravCompensatedArmDesiredDegrees(horizontalDistance, verticalDistance, estimatedExitVelocity, false);
        }
        if (target == FieldElement.LOB_TARGET) {
            // lob shot
            desiredArmAngle = 35;//getGravCompensatedArmDesiredDegrees(horizontalDistance, verticalDistance, estimatedExitVelocity, true);
        }
        if (target == FieldElement.CARPET) {
            // shart
            desiredArmAngle = getSimpleArmDesiredDegrees(horizontalDistance, verticalDistance);
        }
        arm.setDesiredDegrees(desiredArmAngle);

        setpointsAreFresh = true;
    }

    public boolean readyToShoot() {
        return setpointsAreFresh && arm.isCloseToTarget() && flywheels.flywheelsAtSetpoints() && drivetrain.isAligned();
    }

    @Override
    public void end(boolean isInterrupted) {
        ledFeedbackCommand.cancel();
        setpointsAreFresh = false;
    }







    /** Calculates the horizontal translational distance from the center of the robot to target.*/
    public double driveDistToTargetMeters() {
        return FlyingCircuitUtils.getDistanceToFieldElement(target, drivetrain.getPoseMeters());
    }

    public double armDistToTargetMeters() {
        // could do more complex calculation that takes drive-angle into account,
        // but that's overkill because the drive should be aligned with the target
        // quickly anyway, in which case this simple calculation gives the right answer.
        return this.driveDistToTargetMeters() + FieldConstants.pivotOffsetMeters;
    }

    public double getTargetHeight(FieldElement target) {
        if (target == FieldElement.SPEAKER) {
            return FieldConstants.speakerHeightMeters;
        }
        else {
            return 0; // lob shot, shart
        }
    }

    /** Calculates the angle the arm would aim at when pointing directly at the target. */
    public double getSimpleArmDesiredDegrees(double horizontalDistance, double verticalDistance) {
        double radians = Math.atan2(verticalDistance, horizontalDistance); // prob don't need arctan2 here, regular arctan will do.
        return Math.toDegrees(radians);
    }

    /**
     * Gets the angle that the shooter needs to aim at in order for a note to hit the target
     * This accounts for distance and gravity.
     * @return - Angle in degrees, with 0 being straight forward and a positive angle being pointed upwards.
    */
    public double getGravCompensatedArmDesiredDegrees(double horizontalDistance, double verticalDistance, double exitVelocityMetersPerSecond, boolean hitTargetFromAbove) {
        
        //see https://www.desmos.com/calculator/czxwosgvbz

        double h = verticalDistance;
        double d = horizontalDistance;
        double v = exitVelocityMetersPerSecond;
        double g = 9.81;

        double a = (h*h)/(d*d)+1;
        double b = -2*(h*h)*(v*v)/(d*d) - (v*v) - g*h;
        double c = (h*h)*Math.pow(v, 4)/(d*d) + (g*g)*(d*d)/4 + g*h*(v*v);

        // vyBelow will hit the target from below, with the projectile traveling upwards
        // (like the speaker shot)
        // vyAbove will hit the target from above, with the projectile traveling downwards
        // (like the lob shot)
        double vyAbove = Math.sqrt((-b + Math.sqrt(b*b-4*a*c))/(2*a));
        double vyBelow = Math.sqrt((-b - Math.sqrt(b*b-4*a*c))/(2*a));

        if (hitTargetFromAbove) {
            return Math.toDegrees(Math.asin(vyAbove/v));
        }
        else {
            return Math.toDegrees(Math.asin(vyBelow/v));
        }
    }
}
