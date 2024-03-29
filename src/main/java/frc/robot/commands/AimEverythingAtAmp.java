package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FlyingCircuitUtils;
import frc.robot.Constants.FieldElement;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.leds.LEDs;
import frc.robot.subsystems.shooter.Shooter;

public class AimEverythingAtAmp extends Command {

    private Drivetrain drivetrain;
    private Arm arm;
    private Shooter flywheels;
    private Supplier<ChassisSpeeds> translationController;
    private Command ledFeedbackCommand;

    private boolean testingWithoutTags = false;
    private boolean setpointsAreFresh = false;
    private boolean useAutoRotate = false;


    
    /** We are just using manual control for the time being. */
    public AimEverythingAtAmp(boolean useAutoRotate, Drivetrain drivetrain, Arm arm, Shooter flywheels, Supplier<ChassisSpeeds> translationController, LEDs leds) {
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

        if (useAutoRotate) {
            ledFeedbackCommand = leds.playAimingAnimationCommand(arm::getErrorDegrees, flywheels::getWorstError, drivetrain::getAngleError);
        }
        else {
            ledFeedbackCommand = leds.playAimingAnimationCommand(arm::getErrorDegrees, flywheels::getWorstError, () -> {
                Rotation2d desiredAngle = FlyingCircuitUtils.getAngleToFieldElement(FieldElement.AMP, drivetrain.getPoseMeters()).rotateBy(Rotation2d.fromDegrees(180));
                Rotation2d measuredAngle = drivetrain.getPoseMeters().getRotation();
                return desiredAngle.minus(measuredAngle).getDegrees();
            });
        }
    }

    public void initialize() {
        if (testingWithoutTags) {
            Translation2d ampLocation = FlyingCircuitUtils.getLocationOfFieldElement(FieldElement.AMP).getTranslation();
            Translation2d offset = new Translation2d(0, -2);
            Translation2d inFrontOfAmp = ampLocation.plus(offset);
            Rotation2d backPointedAtAmp = Rotation2d.fromDegrees(-90);

            drivetrain.setPoseMeters(new Pose2d(inFrontOfAmp, backPointedAtAmp));
        }

        // drive angle error may be stale from last call?
        // TODO: look into this.
        ledFeedbackCommand.schedule();
        setpointsAreFresh = false;
    }

    public void execute() {
        // Drivetrain
        ChassisSpeeds desiredTranslationalSpeeds = translationController.get();
        if (useAutoRotate) {
            // point the back of the robot at the amp.
            Rotation2d desiredAngle = FlyingCircuitUtils.getAngleToFieldElement(FieldElement.AMP, drivetrain.getPoseMeters()).rotateBy(Rotation2d.fromDegrees(180));
            drivetrain.fieldOrientedDriveWhileAiming(desiredTranslationalSpeeds, desiredAngle);
        }
        else {
            // full manual control
            drivetrain.fieldOrientedDrive(desiredTranslationalSpeeds, true);
        }

        // Flywheels
        double flywheelSpeed = 15;
        if (testingWithoutTags) {
            flywheels.setBothFlywheelsMetersPerSecond(0);
        } else {
            flywheels.setBothFlywheelsMetersPerSecond(flywheelSpeed);
        }

        // Arm
        arm.setDesiredDegrees(110);

        setpointsAreFresh = true;
    }

    public boolean readyToShoot() {
        return setpointsAreFresh && arm.isCloseToTarget() && flywheels.flywheelsAtSetpoints() && drivetrain.isAligned();
    }

    public void end(boolean isInterrupted) {
        ledFeedbackCommand.cancel();
    }
}
