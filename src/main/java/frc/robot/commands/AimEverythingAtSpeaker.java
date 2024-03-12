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

    private boolean useDrivetrain;
    private boolean testingWithoutTags = false;
    public boolean setpointsAreFresh = false;

    /** Command to aim all parts of the robot at the speaker in preparation for a shot.
     *  This command never finishes; instead use the readyToShoot() method to determine when
     *  a shot should be fired. This command also provides the ability to control translation
     *  as the robot aims.
     * 
     *  @param useDrivetrain - True if the drivetrain is able to be controlled by this command. Set this to false when prepping a shot on the move in auto.
     *  This means that the drivetrain will not move at all, only the arm and flywheels. 
     *  @param translationController - Supplier that provides chassisSpeeds so that the robot can be controlled while aiming. If useDrivetrain is false, this value is not used.
     */
    public AimEverythingAtSpeaker(boolean useDrivetrain, Drivetrain drivetrain, Arm arm, Shooter flywheels, Supplier<ChassisSpeeds> translationController, LEDs leds) {
        this.drivetrain = drivetrain;
        this.arm = arm;
        this.flywheels = flywheels;
        this.translationController = translationController;
        this.useDrivetrain = useDrivetrain;
        if (useDrivetrain) {
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

    @Override
    public void execute() {
        // Drivetrain
        if (useDrivetrain) {
            ChassisSpeeds desiredTranslationalSpeeds = translationController.get();
            drivetrain.fieldOrientedDriveWhileAiming(desiredTranslationalSpeeds, drivetrain.getDriveDesiredDegrees());
        }

        // Flywheels
        double leftFlywheelMetersPerSecond = 25;
        double rightFlywheelMetersPerSecond = 15;
        if (testingWithoutTags) {
            flywheels.setBothFlywheelsMetersPerSecond(0);
        } else {
            flywheels.setLeftFlywheelsMetersPerSecond(leftFlywheelMetersPerSecond);
            flywheels.setRightFlywheelsMetersPerSecond(rightFlywheelMetersPerSecond);
        }

        // Arm
        //arm.setDesiredDegrees(this.getSimpleArmDesiredDegrees());
        double estimatedExitVelocity = (leftFlywheelMetersPerSecond + rightFlywheelMetersPerSecond) / 2.0;
        arm.setDesiredDegrees(drivetrain.getGravCompensatedArmDesiredDegrees(estimatedExitVelocity));
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




    
}
