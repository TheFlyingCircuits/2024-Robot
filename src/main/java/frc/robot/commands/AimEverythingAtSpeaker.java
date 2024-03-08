package frc.robot.commands;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;
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

    private boolean testingWithoutTags = false;
    public boolean setpointsAreFresh = false;

    public AimEverythingAtSpeaker(Drivetrain drivetrain, Arm arm, Shooter flywheels, Supplier<ChassisSpeeds> translationController, LEDs leds) {
        this.drivetrain = drivetrain;
        this.arm = arm;
        this.flywheels = flywheels;
        this.translationController = translationController;
        super.addRequirements(drivetrain, arm, flywheels);

        ledFeedbackCommand = leds.playAimingAnimationCommand(arm::getErrorDegrees, flywheels::getWorstError, drivetrain::getAngleError);
    }

    public void initialize() {
        if (testingWithoutTags) {
            Translation2d speakerLocation = FieldConstants.blueSpeakerTranslation2d;
            Translation2d offset = new Translation2d(2, 0);
            Translation2d inFrontOfSpeaker = speakerLocation.plus(offset);

            Rotation2d robotAngle = drivetrain.getPoseMeters().getRotation();
            drivetrain.setPoseMeters(new Pose2d(inFrontOfSpeaker, robotAngle));
        }

        // drive angle error may be stale from last call?
        // TODO: look into this.
        ledFeedbackCommand.schedule();
        setpointsAreFresh = false;
    }

    public void execute() {
        // Drivetrain
        ChassisSpeeds desiredTranslationalSpeeds = translationController.get();
        drivetrain.fieldOrientedDriveWhileAiming(desiredTranslationalSpeeds, this.getDriveDesiredDegrees());

        // Flywheels
        double leftFlywheelMetersPerSecond = 20;
        double rightFlywheelMetersPerSecond = 25;
        if (testingWithoutTags) {
            flywheels.setBothFlywheelsMetersPerSecond(0);
        } else {
            flywheels.setLeftFlywheelsMetersPerSecond(leftFlywheelMetersPerSecond);
            flywheels.setRightFlywheelsMetersPerSecond(rightFlywheelMetersPerSecond);
        }

        // Arm
        //arm.setDesiredDegrees(this.getSimpleArmDesiredDegrees());
        arm.setDesiredDegrees(this.getGravCompensatedArmDesiredDegrees(
            (leftFlywheelMetersPerSecond+rightFlywheelMetersPerSecond)/2));
        // TODO: use measured avg of flywheel speed?

        setpointsAreFresh = true;
    }

    public boolean readyToShoot() {
        return setpointsAreFresh && arm.isCloseToTarget() && flywheels.flywheelsAtSetpoints() && drivetrain.isAligned();
        // TODO: add velocity constraints (can't be passing thorugh setpoint super fast, you must be setteled).
    }

    public void end(boolean isInterrupted) {
        ledFeedbackCommand.cancel();
    }

    public Translation2d getSpeakerLocation() {
        return DriverStation.getAlliance().get() == Alliance.Red ?
               FieldConstants.redSpeakerTranslation2d :
               FieldConstants.blueSpeakerTranslation2d;
    }


    /** Calculates the horizontal translational distance from the center of the robot to the center of the front edge of the speaker.*/
    public double driveDistToSpeakerBaseMeters() {
        Translation2d speakerLocation = this.getSpeakerLocation();
        Translation2d robotLocation = drivetrain.getPoseMeters().getTranslation();
        return robotLocation.getDistance(speakerLocation);
    }

    public double armDistToSpeakerBaseMeters() {
        return this.driveDistToSpeakerBaseMeters() + FieldConstants.pivotOffsetMeters;
    }

    public double getSimpleArmDesiredDegrees() {
        double horizontalDistance = this.armDistToSpeakerBaseMeters();
        double verticalDistance = FieldConstants.speakerHeightMeters - FieldConstants.pivotHeightMeters;
        double radians = Math.atan2(verticalDistance, horizontalDistance); // prob don't need arctan2 here, regular arctan will do.
        return Math.toDegrees(radians);
    }

    /**
     * Gets the angle that the shooter needs to aim at in order to point at the speaker target.
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

    /** TODO: documentation */
    public double getDriveDesiredDegrees() {
        Translation2d speakerLocation = null;

        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (!alliance.isPresent()) {
            // Don't turn if we can't tell where to aim
            return drivetrain.getPoseMeters().getRotation().getDegrees();
        }

        if (alliance.get() == Alliance.Blue) {
            speakerLocation = FieldConstants.blueSpeakerTranslation2d;
        }
        else if (alliance.get() == Alliance.Red) {
            speakerLocation = FieldConstants.redSpeakerTranslation2d;
        }

        // We want robot to align with the vector from robot to speaker
        // that means we want the robot's angle to be the same as that vector's angle
        Translation2d vector = speakerLocation.minus(drivetrain.getPoseMeters().getTranslation());
        return vector.getAngle().getDegrees();
    }


    
}
