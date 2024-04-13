// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.FieldElement;
import frc.robot.Constants.LEDConstants;
import frc.robot.commands.MeasureWheelDiameter;
import frc.robot.commands.UnderStageTrapRoutine;
import frc.robot.commands.PrepShot;
import frc.robot.commands.TrapRoutine;
import frc.robot.subsystems.HumanDriver;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIONeo;
import frc.robot.subsystems.arm.ArmIOSim;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.GyroIOPigeon;
import frc.robot.subsystems.drivetrain.GyroIOSim;
import frc.robot.subsystems.drivetrain.SwerveModuleIOKraken;
import frc.robot.subsystems.drivetrain.SwerveModuleIOSim;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.leds.LEDs;
import frc.robot.subsystems.shooter.Indexer;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIOKraken;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonLib;

import edu.wpi.first.wpilibj.util.Color;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic shoud actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instlead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

    public final HumanDriver charlie = new HumanDriver(0);
    public final CommandXboxController ben = new CommandXboxController(1);

    public final Shooter shooter;
    public final Drivetrain drivetrain;
    public final Arm arm;
    public final Intake intake;
    public final Indexer indexer;
    public final Climb climb;
    public final LEDs leds;
    
    
    public RobotContainer() {

        /**** INITIALIZE SUBSYSTEMS ****/
        if (RobotBase.isReal()) {
            drivetrain = new Drivetrain(
                new GyroIOPigeon(),
                new SwerveModuleIOKraken(3, 1, -0.00342, 7, false, false, "frontLeft"),
                new SwerveModuleIOKraken(2, 2, 0.36816, 6, false, false, "frontRight"),
                new SwerveModuleIOKraken(1, 5, -0.09009, 5, false, true, "backLeft"),
                new SwerveModuleIOKraken(0, 6, -0.37622, 4, true, false, "backRight"),
                new VisionIOPhotonLib()
            );

            /****** FOR NOODLE *******/
            // drivetrain = new Drivetrain(
            //     new GyroIOPigeon(),
            //     new SwerveModuleIONeo(1, 2, -0.177978515625, 0),
            //     new SwerveModuleIONeo(3, 4, 0.33935546875, 1),
            //     new SwerveModuleIONeo(5, 6, -0.339599609375, 2),
            //     new SwerveModuleIONeo(7, 8, -0.206787109375, 3),
            //     new VisionIOPhotonLib()
            // );

            shooter = new Shooter(new ShooterIOKraken());

            arm = new Arm(new ArmIONeo());

        }
        else {

            drivetrain = new Drivetrain(
                new GyroIOSim(),
                new SwerveModuleIOSim(),
                new SwerveModuleIOSim(),
                new SwerveModuleIOSim(),
                new SwerveModuleIOSim(),
                new VisionIO() {}
            );

            shooter = new Shooter(new ShooterIOSim());

            arm = new Arm(new ArmIOSim());
        }

        intake = new Intake();
        indexer = new Indexer();
        climb = new Climb();
        leds = new LEDs();
        
        
        drivetrain.setDefaultCommand(drivetrain.run(() -> {drivetrain.fieldOrientedDrive(charlie.getRequestedFieldOrientedVelocity(), true);}));
        leds.setDefaultCommand(leds.heartbeatCommand().ignoringDisable(true));
        intake.setDefaultCommand(Commands.run(() -> {intake.setVolts(0);}, intake));
        indexer.setDefaultCommand(indexer.setBlackRollerSurfaceSpeedCommand(0));
        shooter.setDefaultCommand(shooter.setFlywheelSurfaceSpeedCommand(0));
        climb.setDefaultCommand(Commands.run(() -> {climb.setVolts(0);}, climb));
        arm.setDefaultCommand(arm.holdCurrentPositionCommand().ignoringDisable(true));


        Trigger noteIsTooFarForPickupInAuto = new Trigger(() -> {
            if (!drivetrain.intakeSeesNote()) {
                return false; // keep driving if you don't see anything
            }

            Optional<Alliance> alliance  = DriverStation.getAlliance();
            if (!alliance.isPresent()) {
                return true; // stop driving if we don't know what alliance we're on
            }

            double noteX = drivetrain.getNearestNoteLocation().getX();
            double midlineX = FieldConstants.midField.getX();
            double overshootAllowance = 1.0; // TODO: tune!

            boolean over = (alliance.get() == Alliance.Blue) && (noteX >= (midlineX + overshootAllowance));
            over = over || (alliance.get() == Alliance.Red) && (noteX <= (midlineX - overshootAllowance));
            return over;
        });


        
        NamedCommands.registerCommand("prepShot", prepAutoSpeakerShot());
        NamedCommands.registerCommand("shootFromAnywhere", speakerShot());
        NamedCommands.registerCommand("waitIndexNote", indexNote().withTimeout(4));
        NamedCommands.registerCommand("indexNote", indexNote().withTimeout(2));
        NamedCommands.registerCommand("intakeNote", intakeNote().withTimeout(1.5));
        NamedCommands.registerCommand("rapidFire", prepAutoSpeakerShot().alongWith(runIntake()));
        NamedCommands.registerCommand("resetShooter", resetShooter());
        NamedCommands.registerCommand("intakeTowardsNote", intakeTowardsNote().until(noteIsTooFarForPickupInAuto).withTimeout(2.5));
        NamedCommands.registerCommand("fireNote", fireNote());


        realBindings();
    }


    //these command compositions must be separated into their own methods
    //since wpilib requires a new instance of a command to be used in each
    //composition (the same instance of a command cannot be used in multiple compositions).

    /**** INTAKE/INDEX ****///////////////////////////////////////////////////////////////////////

    /**
     * @param rapidFire - whether or not we are rapid firing (in auto); if we are, we want to run the indexer faster.
     */
    private Command runIntake() {
        return indexer.setOrangeWheelsSurfaceSpeedCommand(2.5)
                      .alongWith(intake.setVoltsCommand(12));
    }

    private Command reverseIntake() {
        // return indexer.run(() -> {indexer.setVolts(-8);})
        //        .alongWith(intake.setVoltsCommand(-8));
               // TODO: reverse LEDs when barfing?
        return intake.setVoltsCommand(-8);
    }

    /** Starts running the intake, and finishes when a note is detected by the bottom intake sensor.
     *  (or the indexer sensor, for when the note moves so fast past the intake sensor that it doesn't trigger)
     *  Intended to be followed up by indexNote, which will bring the note the rest of the way
     *  into the shooter until it hits the second intake/indexer sensor.
     *  Having intakeNote() be seperate from indexNote() will be usefull for auto,
     *  where we often want to start driving away from a note once we've acquired it,
     *  even if it isn't fully indexed yet.
     * @return
     */
    private Command intakeNote() {
        return new ScheduleCommand(leds.playIntakeAnimationCommand(drivetrain::intakeSeesNote))
        .alongWith(this.runIntake().until(intake::ringJustEnteredIntake));
    }

    private Command indexNote() {
        return this.runIntake().until(indexer::isNoteIndexed)
               .andThen(new InstantCommand(() -> {indexer.setVolts(0); intake.setVolts(0);})
                        .alongWith(new ScheduleCommand(leds.solidOrangeCommand()))
               );
    }
    


    /**** ARM/SHOOTER ****///////////////////////////////////////////////////////////////////////

    /** Resets the angle and speed of the shooter back to its default idle position. */
    private Command resetShooter() {
        double desiredAngle = ArmConstants.armMinAngleDegrees+5; // puts the arm at min height to pass under stage
        // desiredAngle = 35;
        return arm.setDesiredDegreesCommand(desiredAngle)
               .alongWith(shooter.setFlywheelSurfaceSpeedCommand(0));
    }

    /** Moves the arm back and spins up the flywheels to prepare for an amp shot. */
    Command prepAmpShot() {
        Command autoAlignAmpShot = new PrepShot(drivetrain, arm, shooter, charlie::getRequestedFieldOrientedVelocity, leds, FieldElement.AMP);
        Command noAlignAmpShot = new PrepShot(drivetrain, arm, shooter, null, leds, FieldElement.AMP);
        return new ConditionalCommand(autoAlignAmpShot, noAlignAmpShot, drivetrain::inAmpShotRange);
    }

    Command prepLobShot() {
        return new PrepShot(drivetrain, arm, shooter, charlie::getRequestedFieldOrientedVelocity, leds, FieldElement.LOB_TARGET);
    }

    Command shart() {
        PrepShot aim = new PrepShot(drivetrain, arm, shooter, null, leds, FieldElement.CARPET);
        Command waitForAlignment = new WaitUntilCommand(aim::readyToShoot);
        Command fire = fireNote();
        return aim.raceWith(waitForAlignment.andThen(fire));
    }

    public Command speakerShot() {
        PrepShot aim = new PrepShot(drivetrain, arm, shooter, charlie::getRequestedFieldOrientedVelocity, leds, FieldElement.SPEAKER);
        Command waitForAlignment = new WaitUntilCommand(aim::readyToShoot);
        Command fire = fireNote();
        return aim.raceWith(waitForAlignment.andThen(fire));
    }
    

    /**** AUTO ****///////////////////////////////////////////////////////////////////////

    /**
     * Aims the flywheels and arm at the speaker, while also setting the Pathplanner rotation override.
     * This command never finishes.
     */
    private Command prepAutoSpeakerShot() {
        return new InstantCommand(() -> {drivetrain.isTrackingSpeakerInAuto = true;})
                .andThen(new PrepShot(drivetrain, arm, shooter, null, leds, FieldElement.SPEAKER))
                .finallyDo(() -> {drivetrain.isTrackingSpeakerInAuto = false;});

    }

    private Command intakeTowardsNote() {
        // keep charlie control for auto? he will stop, which is what we want if nothing is seen?
        return drivetrain.run(() -> {drivetrain.driveTowardsNote(charlie::getRequestedFieldOrientedVelocity);})
               .raceWith(intakeNote());
    }


    private Command pathfindToNote(FieldElement note) {

        Pose2d targetPose = FlyingCircuitUtils.pickupAtNote(
                drivetrain.getPoseMeters().getTranslation(),
                note.getLocation().toTranslation2d(), 
                0.);


        return AutoBuilder.pathfindToPose(
                targetPose,
                DrivetrainConstants.pathfindingConstraints,
                0.
            );
    }


    private Command intakeAndThenShoot(Pose2d scoringPose) {
        //TODO: figure out how to navigate to nearest scoring location instead of a preset one
        //use lambdas or something for scoringPose
        
        return new SequentialCommandGroup(
            intakeTowardsNote(),
            new ParallelDeadlineGroup(
                AutoBuilder.pathfindToPose(scoringPose, DrivetrainConstants.pathfindingConstraints),
                prepAutoSpeakerShot()
            ),
            speakerShot()
        );
    }

    public Command pathfindingAuto(List<FieldElement> targetNotes, Pose2d scoringPose) {

        

        Command autoCommand = new InstantCommand();

        for (int noteInd = 0; noteInd < targetNotes.size(); noteInd++) {
            autoCommand = autoCommand
                .andThen(pathfindToNote(targetNotes.get(noteInd)).deadlineWith(resetShooter()))
                .andThen(intakeAndThenShoot(scoringPose).onlyIf(drivetrain::intakeSeesNote));
        }

        return autoCommand;
    }
    
    public Command fireNote() {
        return indexer.setOrangeWheelsSurfaceSpeedCommand(7).withTimeout(0.1)
               .alongWith(new ScheduleCommand(leds.playFireNoteAnimationCommand()));
    }

    public Command fireNoteThroughHood() {
        // The flywheels need to run longer than normal to make sure the note
        // makes it all the way through the slam dunk hood.
        return indexer.setOrangeWheelsSurfaceSpeedCommand(7).withTimeout(0.5)
               .alongWith(new ScheduleCommand(leds.playFireNoteAnimationCommand()));
    }


    private void realBindings() {
        CommandXboxController controller = charlie.getXboxController();
        /** INTAKE **/
        controller.rightTrigger()
            .onTrue(intakeTowardsNote().andThen(new ScheduleCommand(
                indexNote().andThen(reverseIntake().withTimeout(2))
            )));
            //.onTrue(intakeNote().andThen(indexNote()));
            //.onTrue(indexNote().raceWith(resetShooter())); // reset never ends, indexNote does.
    
        controller.leftTrigger().whileTrue(reverseIntake().alongWith(indexer.run(() -> {indexer.setVolts(-8);})));
        
        
        /** SCORING **/
        
        //speaker shot
        Trigger inSpeakerShotRange = new Trigger(drivetrain::inSpeakerShotRange);
        controller.rightBumper().and(inSpeakerShotRange)
            .onTrue(
                this.speakerShot()
                .andThen(new ScheduleCommand(this.resetShooter()))
            );

        //lob shot (prep while holding, release to fire)
        Command prepLobShot = prepLobShot(); // <- grab a single instance so we can check if it's cancelled.
        controller.rightBumper().and(inSpeakerShotRange.negate())
            .onTrue(prepLobShot)
            .onFalse(this.fireNote()
                         .andThen(new ScheduleCommand(this.resetShooter()))
                     .unless(() -> {return !prepLobShot.isScheduled();})
                     // only fire if the lob shot wasn't cancelled
            );
            


        controller.leftBumper()
            .onTrue(prepAmpShot())
            .onFalse(this.fireNoteThroughHood().andThen(new ScheduleCommand(this.resetShooter())));
            // use a schedule command so the onFalse sequence doesn't cancel the aiming while the note is being shot.
            
        controller.b().onTrue(shart().andThen(new ScheduleCommand(this.resetShooter())));
        // TODO: raise shart a tad

        /** CLIMB **/
        
        controller.povUp().onTrue(climb.raiseHooksCommand());
        controller.povRight().onTrue(climb.lowerHooksCommand().until((climb::climbArmsZero)));
        controller.povDown().whileTrue(climb.lowerHooksCommand().until(climb::atQuickClimbSetpoint));
        controller.a().whileTrue(new UnderStageTrapRoutine(charlie::getRequestedFieldOrientedVelocity, climb, arm, shooter, drivetrain, this::fireNoteThroughHood));

        //controller.povLeft().onTrue(arm.setDesiredDegreesCommand(ArmConstants.armMaxAngleDegrees));

        /** MISC **/
        //controller.y().onTrue(new InstantCommand(() -> drivetrain.setPoseToVisionMeasurement()).repeatedly().until(drivetrain::seesTag));
        // ben.y().onTrue(new InstantCommand(() -> drivetrain.setPoseToVisionMeasurement()));
        controller.y().onTrue(new InstantCommand(() -> drivetrain.setRobotFacingForward()));

        controller.x().onTrue(new InstantCommand(() -> arm.setDisableSetpointChecking(false)).andThen(resetShooter()));

        controller.start().whileTrue(new MeasureWheelDiameter(drivetrain));

        // controller.back().whileTrue(
        //     pathfindingAuto(
        //         Arrays.asList(
        //             FieldElement.NOTE_1,
        //             FieldElement.NOTE_2,
        //             FieldElement.NOTE_3
        //         ),
        //         new Pose2d(12, 6.5, new Rotation2d())));

        /** Driver Feedback **/
        Trigger ringJustEnteredIntake = new Trigger(intake::ringJustEnteredIntake);
        ringJustEnteredIntake.onTrue(charlie.rumbleController(0.5).withTimeout(0.25)); // lol this happens even during auto
        ringJustEnteredIntake.onTrue(leds.temporarilySwitchPattern(leds.strobeCommand(Color.kWhite, 4, 0.5).ignoringDisable(true)).ignoringDisable(true));
        // TODO: prevent flash on reverse? Either condition with positive wheel speeds,
        //       or no seperate schedule command?
    }
}
