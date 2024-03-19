package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FlyingCircuitUtils;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.FieldElement;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.leds.LEDs;
import frc.robot.subsystems.shooter.Shooter;

public class AimEverythingAtSpeaker extends Command {


    private Drivetrain drivetrain;
    private Arm arm;
    private Shooter flywheels;
    private Supplier<ChassisSpeeds> translationController;
    private Command ledFeedbackCommand;

    private boolean useAutoRotate;
    private boolean testingWithoutTags = false;
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
    public AimEverythingAtSpeaker(boolean useAutoRotate, Drivetrain drivetrain, Arm arm, Shooter flywheels, Supplier<ChassisSpeeds> translationController, LEDs leds) {
        this.drivetrain = drivetrain;
        this.arm = arm;
        this.flywheels = flywheels;
        this.translationController = translationController;
        this.useAutoRotate = useAutoRotate;
        if (useAutoRotate) {
            super.addRequirements(drivetrain, arm, flywheels);
        }
        else {
            super.addRequirements(arm, flywheels);
        }
        

        ledFeedbackCommand = leds.playAimingAnimationCommand(arm::getErrorDegrees, flywheels::getWorstError, drivetrain::getAngleError);
    }

    @Override
    public void initialize() {
        if (testingWithoutTags) {
            Translation2d speakerLocation = FieldConstants.blueSpeakerTranslation2d;
            Translation2d offset = new Translation2d(2.5, 0);
            Translation2d inFrontOfSpeaker = speakerLocation.plus(offset);

            Rotation2d robotAngle = drivetrain.getPoseMeters().getRotation();
            drivetrain.setPoseMeters(new Pose2d(inFrontOfSpeaker, robotAngle));
        }

        // drive angle error may be stale from last call?
        // TODO: look into this.
        ledFeedbackCommand.schedule();
        setpointsAreFresh = false;
    }

    @Override
    public void execute() {
        // Drivetrain
        if (useAutoRotate) {
            ChassisSpeeds desiredTranslationalSpeeds = translationController.get();
            Rotation2d desiredAngle = FlyingCircuitUtils.getAngleToFieldElement(FieldElement.SPEAKER, drivetrain.getPoseMeters());
            drivetrain.fieldOrientedDriveWhileAiming(desiredTranslationalSpeeds, desiredAngle);
        }

        // Flywheels
        double leftFlywheelMetersPerSecond = 25;
        double rightFlywheelMetersPerSecond = 20;
        if (testingWithoutTags) {
            flywheels.setBothFlywheelsMetersPerSecond(0);
        } else {
            flywheels.setLeftFlywheelsMetersPerSecond(leftFlywheelMetersPerSecond);
            flywheels.setRightFlywheelsMetersPerSecond(rightFlywheelMetersPerSecond);
        }

        // Arm
        //arm.setDesiredDegrees(this.getSimpleArmDesiredDegrees());
        double estimatedExitVelocity = (leftFlywheelMetersPerSecond + rightFlywheelMetersPerSecond) / 2.0;
        arm.setDesiredDegrees(getGravCompensatedArmDesiredDegrees(estimatedExitVelocity));
        // TODO: use measured avg of flywheel speed?

        setpointsAreFresh = true;
    }

    public boolean readyToShoot() {
        return setpointsAreFresh && arm.isCloseToTarget() && flywheels.flywheelsAtSetpoints() && drivetrain.isAligned();
    }

    @Override
    public void end(boolean isInterrupted) {
        ledFeedbackCommand.cancel();
    }







/** Calculates the horizontal translational distance from the center of the robot to the central apriltag of the speaker.*/
public double driveDistToSpeakerBaseMeters() {
    Translation2d speakerLocation = FlyingCircuitUtils.getLocationOfFieldElement(FieldElement.SPEAKER).getTranslation();
    Translation2d robotLocation = drivetrain.getPoseMeters().getTranslation();
    return robotLocation.getDistance(speakerLocation);
}

public double armDistToSpeakerBaseMeters() {
    return this.driveDistToSpeakerBaseMeters() + FieldConstants.pivotOffsetMeters;
}

/** Calculates the angle the arm would aim at to make a straight line to the speaker target. */
public double getSimpleArmDesiredDegrees() {
    double horizontalDistance = this.armDistToSpeakerBaseMeters();
    double verticalDistance = FieldConstants.speakerHeightMeters - FieldConstants.pivotHeightMeters;
    double radians = Math.atan2(verticalDistance, horizontalDistance); // prob don't need arctan2 here, regular arctan will do.
    return Math.toDegrees(radians);
}

/**
 * Gets the angle that the shooter needs to aim at in order for a note to make it into the speaker.
 * This accounts for distance and gravity.
 * @return - Angle in degrees, with 0 being straight forward and a positive angle being pointed upwards.
*/
public double getGravCompensatedArmDesiredDegrees(double exitVelocityMetersPerSecond) {
    
    //see https://www.desmos.com/calculator/czxwosgvbz

    double h = FieldConstants.speakerHeightMeters-FieldConstants.pivotHeightMeters;
    double d = this.armDistToSpeakerBaseMeters();
    double v = exitVelocityMetersPerSecond;
    double g = 9.81;

    double a = (h*h)/(d*d)+1;
    double b = -2*(h*h)*(v*v)/(d*d) - (v*v) - g*h;
    double c = (h*h)*Math.pow(v, 4)/(d*d) + (g*g)*(d*d)/4 + g*h*(v*v);

    double vy = Math.sqrt((-b-Math.sqrt(b*b-4*a*c))/(2*a));

    return Math.toDegrees(Math.asin(vy/v));
}




    
}
