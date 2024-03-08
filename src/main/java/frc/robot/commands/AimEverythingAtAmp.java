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

public class AimEverythingAtAmp extends Command {

    private Drivetrain drivetrain;
    private Arm arm;
    private Shooter flywheels;
    private Supplier<ChassisSpeeds> translationController;
    private Command ledFeedbackCommand;

    private boolean testingWithoutTags = true;
    public boolean setpointsAreFresh = false;

    public AimEverythingAtAmp(Drivetrain drivetrain, Arm arm, Shooter flywheels, Supplier<ChassisSpeeds> translationController, LEDs leds) {
        this.drivetrain = drivetrain;
        this.arm = arm;
        this.flywheels = flywheels;
        this.translationController = translationController;
        super.addRequirements(drivetrain, arm, flywheels);

        ledFeedbackCommand = leds.playAimingAnimationCommand(arm::getErrorDegrees, flywheels::getWorstError, drivetrain::getAngleError);
    }

    public void initialize() {
        if (testingWithoutTags) {
            Translation2d ampLocation = FieldConstants.blueAmpLocation;
            Translation2d offset = new Translation2d(0, -2);
            Translation2d inFrontOfAmp = ampLocation.plus(offset);

            Rotation2d robotAngle = drivetrain.getPoseMeters().getRotation();
            drivetrain.setPoseMeters(new Pose2d(inFrontOfAmp, robotAngle));
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
        double leftFlywheelMetersPerSecond = 10;
        double rightFlywheelMetersPerSecond = 10;
        if (testingWithoutTags) {
            flywheels.setBothFlywheelsMetersPerSecond(0);
        } else {
            flywheels.setLeftFlywheelsMetersPerSecond(leftFlywheelMetersPerSecond);
            flywheels.setRightFlywheelsMetersPerSecond(rightFlywheelMetersPerSecond);
        }

        // Arm
        arm.setDesiredDegrees(110);

        setpointsAreFresh = true;
    }

    public boolean readyToShoot() {
        return setpointsAreFresh && arm.isCloseToTarget() && flywheels.flywheelsAtSetpoints() && drivetrain.isAligned();
        // TODO: add velocity constraints (can't be passing thorugh setpoint super fast, you must be setteled).
    }

    public void end(boolean isInterrupted) {
        ledFeedbackCommand.cancel();
    }

    public Translation2d getAmpLocation() {
        return DriverStation.getAlliance().get() == Alliance.Red ?
               FieldConstants.redAmpLocation :
               FieldConstants.blueAmpLocation;
    }

    /** TODO: documentation */
    public double getDriveDesiredDegrees() {
        Translation2d ampLocation = null;

        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (!alliance.isPresent()) {
            // Don't turn if we can't tell where to aim
            return drivetrain.getPoseMeters().getRotation().getDegrees();
        }

        if (alliance.get() == Alliance.Blue) {
            ampLocation = FieldConstants.blueAmpLocation;
        }
        else if (alliance.get() == Alliance.Red) {
            ampLocation = FieldConstants.redAmpLocation;
        }

        // We want robot to align with the vector from robot to speaker
        // that means we want the robot's angle to be the same as that vector's angle
        Translation2d vector = ampLocation.minus(drivetrain.getPoseMeters().getTranslation());
        return vector.getAngle().getDegrees();
    }


    
}
