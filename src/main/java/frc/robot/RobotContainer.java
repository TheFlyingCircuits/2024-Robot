// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.FieldElement;
import frc.robot.Constants.LEDConstants;
import frc.robot.commands.MeasureWheelDiameter;
import frc.robot.commands.UnderStageTrapRoutine;
import frc.robot.commands.PrepShot;
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
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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

    private Trigger dontAmpAutoAlign;

    private boolean goodPickup = false;
    
    
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
        // leds.setDefaultCommand(leds.heartbeatCommand(1.5).andThen(leds.heartbeatCommand(1.0)).ignoringDisable(true));
        // leds.setDefaultCommand(leds.fasterHeartbeatSequence().ignoringDisable(true));
        intake.setDefaultCommand(Commands.run(() -> {intake.setVolts(0);}, intake));
        indexer.setDefaultCommand(indexer.setBlackRollerSurfaceSpeedCommand(0));
        shooter.setDefaultCommand(shooter.setFlywheelSurfaceSpeedCommand(0));
        climb.setDefaultCommand(Commands.run(() -> {climb.setVolts(0);}, climb));
        arm.setDefaultCommand(arm.holdCurrentPositionCommand().ignoringDisable(true));

        dontAmpAutoAlign = ben.leftBumper();


        realBindings();
    }

    private void realBindings() {
        // Spring control testing
        // ben.b().whileTrue(shooter.runSpringControl(true));
        // ben.b().whileFalse(shooter.runSpringControl(false));
        // ben.a().whileTrue(arm.runSpringControl(true));
        // ben.a().whileFalse(arm.runSpringControl(false));


        CommandXboxController controller = charlie.getXboxController();
        /** INTAKE **/
        controller.rightTrigger()
            .onTrue(
                //intake after note if on other side of the field
                //new ConditionalCommand(


                    intakeTowardsNote(charlie::getRequestedFieldOrientedVelocity).andThen(new ScheduleCommand(
                    indexNote().andThen(reverseIntake().withTimeout(1.0))))


                    // drivetrain.run(() -> {drivetrain.driveTowardsNote(new Translation2d());})
                    // indexNote()
                // Schedule index, so the drive command goes back to default as soon as intake is done
                //)),
                //regular intake if on this side of the field
                //intakeNote().andThen(indexNote()),
                //() -> {return !drivetrain.inSpeakerShotRange();})
            );



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

        /** CLIMB **/
        
        controller.povUp().onTrue(climb.raiseHooksCommand());
        controller.povRight().onTrue(climb.homeHooksCommand());
        controller.povDown().onTrue(climb.lowerHooksCommand().until(climb::atQuickClimbSetpoint));

        controller.a().whileTrue(new UnderStageTrapRoutine(charlie::getRequestedFieldOrientedVelocity, climb, arm, shooter, drivetrain, this::fireNoteThroughHood))
                .onFalse(new InstantCommand(() -> {drivetrain.useShooterCamera = true;}));


        // controller.a().whileTrue(arm.run(() -> {
        //     double armTorque = SmartDashboard.getNumber("armTorque", 0);
        //     SmartDashboard.putNumber("armTorque", armTorque);
        //     // arm.exertTorque(armTorque);
        //     arm.resistGravity();
        // }))
        // .whileFalse(arm.run(() -> {
        //     arm.setVolts(0);
        // }));

        // controller.povRight().onTrue(arm.setDesiredDegreesCommand(0));
        // controller.povDown().onTrue(arm.setDesiredDegreesCommand(ArmConstants.armMinAngleDegrees));
        // controller.povUp().onTrue(arm.setDesiredDegreesCommand(90));
        // controller.povUpRight().onTrue(arm.setDesiredDegreesCommand(45));
        // controller.povLeft().onTrue(arm.setDesiredDegreesCommand(ArmConstants.armMaxAngleDegrees));

        /** MISC **/
        controller.y().onTrue(new InstantCommand(() -> drivetrain.setPoseToVisionMeasurement()).repeatedly().until(drivetrain::seesTag));
        ben.y().onTrue(new InstantCommand(() -> drivetrain.setPoseToVisionMeasurement()).repeatedly().until(drivetrain::seesTag));
        // controller.y().onTrue(new InstantCommand(() -> drivetrain.setRobotFacingForward()));
        controller.x().onTrue(new InstantCommand(() -> arm.setDisableSetpointChecking(false)).andThen(resetShooter()));

        // controller.start().whileTrue(new MeasureWheelDiameter(drivetrain));
        // controller.start().whileTrue(sourceSideAuto());
        // controller.back().whileTrue(ampSideAuto());

        /** Driver Feedback **/
        Trigger ringJustEnteredIntake = new Trigger(intake::ringJustEnteredIntake);
        ringJustEnteredIntake.onTrue(charlie.rumbleController(0.5).withTimeout(0.25)); // lol this happens even during auto
        ringJustEnteredIntake.onTrue(leds.temporarilySwitchPattern(leds.strobeCommand(Color.kWhite, 4, 0.5).ignoringDisable(true)).ignoringDisable(true));
        // TODO: prevent flash on reverse? Either condition with positive wheel speeds,
        //       or no seperate schedule command?
    }


    //these command compositions must be separated into their own methods
    //since wpilib requires a new instance of a command to be used in each
    //composition (the same instance of a command cannot be used in multiple compositions).

    /**** INTAKE/INDEX ****///////////////////////////////////////////////////////////////////////

    private Command runIntake(boolean isIndexing) {
        if (!isIndexing) {
            return indexer.setOrangeWheelsSurfaceSpeedCommand(2.5)
                          .alongWith(intake.setVoltsCommand(12));
        }
        return indexer.setOrangeWheelsSurfaceSpeedCommand(2.5)
                      .alongWith(intake.setVoltsCommand(12, -0.5));
        // when indexing, slightly spin the first intake wheel that isn't a sushi roller backwards
        // to prevent double intaking when notes are really close to one another.
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
        return new ScheduleCommand(leds.playIntakeAnimationCommand(() -> {return drivetrain.getBestNoteLocationFieldFrame().isPresent();}))
            .alongWith(this.runIntake(false).until(intake::ringJustEnteredIntake));
    }

    /**
     * @param howToDriveWhenNoNoteDetected let's charlie have control if the noteCam doesn't see a note
     * @return
     */
    private Command intakeTowardsNote(Supplier<ChassisSpeeds> howToDriveWhenNoNoteDetected) {
        return intakeNote().raceWith(drivetrain.run(() -> {

            // have charlie stay in control when the noteCam doesn't see a note
            if (drivetrain.getBestNoteLocationFieldFrame().isEmpty()) {
                drivetrain.fieldOrientedDrive(howToDriveWhenNoNoteDetected.get(), true);
                return;
            }

            // drive towards the note when the noteCam does see a note.
            drivetrain.driveTowardsNote(drivetrain.getBestNoteLocationFieldFrame().get());
        }));
    }


    private Command indexNote() {
        return this.runIntake(true).until(indexer::isNoteIndexed)
               .andThen(new InstantCommand(() -> {indexer.setVolts(0); intake.setVolts(0);})
                        .alongWith(new ScheduleCommand(leds.solidOrangeCommand()))
               );
    }
    
        
    private Command fireNote() {
        return indexer.setOrangeWheelsSurfaceSpeedCommand(7).withTimeout(0.1)
               .alongWith(new ScheduleCommand(leds.playFireNoteAnimationCommand()));
    }

    private Command fireNoteThroughHood() {
        // The flywheels need to run longer than normal to make sure the note
        // makes it all the way through the slam dunk hood.
        return indexer.setOrangeWheelsSurfaceSpeedCommand(7).withTimeout(0.5)
               .alongWith(new ScheduleCommand(leds.playFireNoteAnimationCommand()));
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
    private Command prepAmpShot() {
        Command autoAlignAmpShot = new PrepShot(drivetrain, arm, shooter, charlie::getRequestedFieldOrientedVelocity, leds, FieldElement.AMP);
        Command noAlignAmpShot = new PrepShot(drivetrain, arm, shooter, null, leds, FieldElement.AMP);

        return noAlignAmpShot;
        // return new ConditionalCommand(
        //     autoAlignAmpShot,
        //     noAlignAmpShot,
        //     () -> {return drivetrain.inAmpShotRange() && !dontAmpAutoAlign.getAsBoolean();});
        // This was probably spinning because the conditional command as a whole required the drivetrain,
        // even when we chose the noAlignAmpShot. We should use schedule commands inside the conditional instead.
    }

    private Command prepLobShot() {
        return new PrepShot(drivetrain, arm, shooter, charlie::getRequestedFieldOrientedVelocity, leds, FieldElement.LOB_TARGET);
    }

    private Command shart() {
        PrepShot aim = new PrepShot(drivetrain, arm, shooter, null, leds, FieldElement.CARPET);
        Command waitForAlignment = new WaitUntilCommand(aim::readyToShoot);
        Command fire = fireNote();
        return aim.raceWith(waitForAlignment.andThen(fire));
    }

    private Command speakerShot() {
        PrepShot aim = new PrepShot(drivetrain, arm, shooter, charlie::getRequestedFieldOrientedVelocity, leds, FieldElement.SPEAKER);
        Command waitForAlignment = new WaitUntilCommand(() -> {return drivetrain.hasRecentSpeakerTagMeasurement(0.25);})
                                   .andThen(new WaitUntilCommand(aim::readyToShoot));  // wait for vision to stabilize
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

    /**
     * Indexes with a timeout of 2 seconds, and then calls prepAutoSpeakerShot().
     */
    private Command autoIndexAndThenPrep() {
        return indexNote().withTimeout(2).andThen(prepAutoSpeakerShot());
    }


    private Command autoIntakeTowardsNote(FieldElement note) {
        return new SequentialCommandGroup(
            // Assume the pickup didn't work until proven otherwise
            new InstantCommand(() -> {this.goodPickup = false;}),

            // Drive towards the note if we see it, or drive to where it should be if we don't see it.
            intakeNote().raceWith(drivetrain.run(() -> {

                Translation2d noteLocation = note.getLocation().toTranslation2d();
                if (drivetrain.getBestNoteLocationFieldFrame().isPresent()) {
                    noteLocation = drivetrain.getBestNoteLocationFieldFrame().get();
                }

                drivetrain.driveTowardsNote(noteLocation);
            }))
            // Set goodPickup to true if the intake sequence finishies normally
            // (i.e. intakeNote() wins the ParallelRaceGroup because the intake sensor triggered)
            .finallyDo(
                (boolean interrupted) -> {this.goodPickup = !interrupted;}
            )
            // Interrupt the intake sequence if it's determined that the current target note is a lost cause
            // (already taken by the opposing alliance, or too risky to pickup because its so far over the midline that
            // we might get fouls by going for it)
            .until(() -> {return noteIsLostCauseInAuto(note);})
         );
    }

    /**
     * TODO: documentation
     * @param note
     * @return
     */
    private boolean noteIsLostCauseInAuto(FieldElement note) {

        boolean canSeeNote = drivetrain.getBestNoteLocationFieldFrame().isPresent();
        Translation2d noteLocation = note.getLocation().toTranslation2d();
        if (canSeeNote) {
            noteLocation = drivetrain.getBestNoteLocationFieldFrame().get();
        }

        boolean noteIsGone = shouldSeeNote(note) && !canSeeNote;
        boolean pickupTooRisky = noteIsTooRiskyForPickupInAuto(noteLocation);

        // if the first note is too farr away, then that's the note we'll likely see on the next frame
        // even if we're starting to turn away. This is why we need the additional shouldSeeNote() check.
        pickupTooRisky = pickupTooRisky && shouldSeeNote(note);

        Logger.recordOutput("auto/targetNote", note.name());
        Logger.recordOutput("auto/noteIsGone", noteIsGone);
        Logger.recordOutput("auto/pickupTooRisky", pickupTooRisky);

        return noteIsGone || pickupTooRisky;
    }


    private boolean shouldSeeNote(FieldElement note) {
        double pickupRadius = 0.75; // want this to be large for early exit.
        double fovConeDegrees = 3; // need this so we don't abort too quickly.
        Translation2d noteToRobot = drivetrain.getPoseMeters().getTranslation().minus(note.getLocation().toTranslation2d());
        boolean closeEnough = noteToRobot.getNorm() <= pickupRadius;
        boolean noteInFov = Math.abs(drivetrain.getPoseMeters().getRotation().minus(noteToRobot.getAngle()).getDegrees()) <= (fovConeDegrees / 2.);
        return closeEnough && noteInFov;
    }

    private boolean noteIsTooRiskyForPickupInAuto(Translation2d noteLocation) {
        // We'll risk getting fowls if the note is too far into enemy territory.
        double midlineX = FieldElement.MID_FIELD.getX();
        double overshootAllowanceMeters = 2.0; // TODO: tune me!

        boolean tooRiskyForBlue = noteLocation.getX() > (midlineX + overshootAllowanceMeters);
        boolean tooRiskyForRed  = noteLocation.getX() < (midlineX - overshootAllowanceMeters);

        boolean onBlueAlliance = FieldElement.SPEAKER.getLocation().getX() < midlineX;

        return (onBlueAlliance && tooRiskyForBlue) || (!onBlueAlliance && tooRiskyForRed);
    }


    /**
     * Navigates from a scoring location to the pickup spot for the next note.
     * @param targetRing - the note you want to pickup.
     */
    private Command navigatePickupAfterShot(FieldElement note) {
        Command pathFollowingCommand = null;

        if (note == FieldElement.NOTE_4) {
            pathFollowingCommand = FlyingCircuitUtils.followPath("Starting Line to Ring 4");
        }
        if (note == FieldElement.NOTE_5) {
            pathFollowingCommand = FlyingCircuitUtils.followPath("Amp Shot to Ring 5");
        }
        if (note == FieldElement.NOTE_6) {
            // Choose which way to approach note6 based on which side of the field we're on when we make the shot.
            pathFollowingCommand = new ConditionalCommand(
                FlyingCircuitUtils.followPath("Amp Shot to Ring 6"), 
                FlyingCircuitUtils.followPath("Source Shot to Ring 6"), 
                () -> {
                    Translation2d robotLocation = drivetrain.getPoseMeters().getTranslation();
                    boolean onAmpSide = robotLocation.getY() >= FieldElement.MID_FIELD.getY();
                    return onAmpSide;
                }).withName("Shot to Ring 6");
        }
        if (note == FieldElement.NOTE_7) {
            pathFollowingCommand = FlyingCircuitUtils.followPath("Source Shot to Ring 7");
        }
        if (note == FieldElement.NOTE_8) {
            pathFollowingCommand = FlyingCircuitUtils.followPath("Starting Line to Ring 8");
        }

        return new ParallelDeadlineGroup(
                pathFollowingCommand,
                resetShooter(),
                new PrintCommand("Driving to " + note.name())
                );
    }

    /**
     * Navigates from a note to a scoring location after a pickup.
     * @param note - the note you are navigating from.
     */
    private Command scoreRingAfterPickup(FieldElement note) {
        Command pathFollowingCommand = null;

        if (note == FieldElement.NOTE_4) {
            pathFollowingCommand = FlyingCircuitUtils.followPath("Ring 4 to Amp Shot");
        }
        if (note == FieldElement.NOTE_5) {
            pathFollowingCommand = FlyingCircuitUtils.followPath("Ring 5 to Amp Shot");
        }
        if (note == FieldElement.NOTE_6) {
            pathFollowingCommand = new ConditionalCommand(
                FlyingCircuitUtils.followPath("Ring 6 to Amp Shot"),
                FlyingCircuitUtils.followPath("Ring 6 to Source Shot"),
                () -> {
                    boolean kindaFacingAmp = drivetrain.getPoseMeters().getRotation().getSin() > 0;
                    return kindaFacingAmp;
                }
            ).withName("Ring 6 to Shot");
        }
        if (note == FieldElement.NOTE_7) {
            pathFollowingCommand = FlyingCircuitUtils.followPath("Ring 7 to Source Shot");
        }
        if (note == FieldElement.NOTE_8) {
            pathFollowingCommand = FlyingCircuitUtils.followPath("Ring 8 to Source Shot");
        }
        
        return new SequentialCommandGroup(
            new ParallelDeadlineGroup(
                pathFollowingCommand,
                autoIndexAndThenPrep(),
                new PrintCommand("Scoring " + note.name())
            ),
            speakerShot()
        );
    }


    public Command ampSideAuto() {
        return new SequentialCommandGroup(
            speakerShot(),
            navigatePickupAfterShot(FieldElement.NOTE_4),
            autoIntakeTowardsNote(FieldElement.NOTE_4),
            //if you pickup a note, score it and go to ring 5
            //otherwise just go to ring 5
            new ConditionalCommand(
                new SequentialCommandGroup(
                    scoreRingAfterPickup(FieldElement.NOTE_4),
                    navigatePickupAfterShot(FieldElement.NOTE_5)
                ), 
                new PrintCommand("skipping 4, trying 5"),
                () -> {return this.goodPickup;}
            ),
            autoIntakeTowardsNote(FieldElement.NOTE_5),
            //if you pickup a note, score it and go to ring 6
            //otherwise just go to ring 6
            new ConditionalCommand(
                new SequentialCommandGroup(
                    scoreRingAfterPickup(FieldElement.NOTE_5),
                    navigatePickupAfterShot(FieldElement.NOTE_6)
                ), 
                new PrintCommand("skipping 5, trying 6"),
                () -> {return this.goodPickup;}),
            autoIntakeTowardsNote(FieldElement.NOTE_6),
            //if you pickup ring 6, score it, otherwise sit and do nothing
            new ConditionalCommand(
                scoreRingAfterPickup(FieldElement.NOTE_6),
                new InstantCommand(),
                () -> {return this.goodPickup;})

        ).withName("Amp Side HyperChad Auto");
    }

    public Command sourceSideAuto() {
        return new SequentialCommandGroup(
            speakerShot(),
            navigatePickupAfterShot(FieldElement.NOTE_8),
            autoIntakeTowardsNote(FieldElement.NOTE_8),
            //if you pickup ring 8, score it and go to ring 7
            //otherwise just go to ring 7
            new ConditionalCommand(
                new SequentialCommandGroup(
                    scoreRingAfterPickup(FieldElement.NOTE_8),
                    navigatePickupAfterShot(FieldElement.NOTE_7)), 
                new PrintCommand("skipping 8, trying 7"),
                () -> {return this.goodPickup;}),
            autoIntakeTowardsNote(FieldElement.NOTE_7),
            //if you pickup ring 7, score it and go to ring 6
            //otherwise just go to ring 6
            new ConditionalCommand(
                new SequentialCommandGroup(
                    scoreRingAfterPickup(FieldElement.NOTE_7),
                    navigatePickupAfterShot(FieldElement.NOTE_6)
                ), 
                new PrintCommand("skipping 7, trying 6"),
                () -> {return this.goodPickup;}),
            autoIntakeTowardsNote(FieldElement.NOTE_6),
            //if you pickup ring 6, score it, otherwise sit and do nothing
            new ConditionalCommand(
                scoreRingAfterPickup(FieldElement.NOTE_6),
                new InstantCommand(),
                () -> {return this.goodPickup;})

        ).withName("Source Side HyperChad Auto");
    }

    public Command centerSideAuto() {
        return new SequentialCommandGroup(
            // Fire preload
            speakerShot(),

            // Pickup ring 6
            new ParallelDeadlineGroup(
                FlyingCircuitUtils.followPath("Starting Line to Ring 6 (Center Side)"),
                resetShooter()
            ),
            autoIntakeTowardsNote(FieldElement.NOTE_6),

            // Return trip
            new ParallelDeadlineGroup(
                FlyingCircuitUtils.followPath("Ring 6 to Starting Line (Center Side)"),
                indexNote()
            ),

            // Fire ring 6
            speakerShot(),

            // Start rapid fire sequence
            new ParallelRaceGroup(
                // Run intake while aiming
                prepAutoSpeakerShot().alongWith(runIntake(false)),

                // Follow the rapid fire path
                new SequentialCommandGroup(
                    FlyingCircuitUtils.followPath("Starting Line to Ring 3"),
                    FlyingCircuitUtils.followPath("Ring 3 to Ring 2"),
                    FlyingCircuitUtils.followPath("Ring 2 to Ring 1")
                )
            )

        ).withName("(6, 3, 2, 1) 5 Piece Center Side");
    }

    public Command sourceSideAuto(FieldElement[] notesToGoFor) {
        FieldElement highPriorityNote = notesToGoFor[0];
        FieldElement midPriorityNote = notesToGoFor[1];
        FieldElement lowPriorityNote = notesToGoFor[2];

        char highPriorityAsChar = highPriorityNote.name().charAt(highPriorityNote.name().length()-1);
        char midPriorityAsChar = midPriorityNote.name().charAt(midPriorityNote.name().length()-1);
        char lowPriorityAsChar = lowPriorityNote.name().charAt(lowPriorityNote.name().length()-1);
        String autoNamePrefix = "("+highPriorityAsChar+", "+midPriorityAsChar+", "+lowPriorityAsChar+")";

        return new SequentialCommandGroup(
            new ParallelDeadlineGroup(
                FlyingCircuitUtils.followPath("Starting Line to Source Shot"),
                prepAutoSpeakerShot()
            ),
            speakerShot(),
            navigatePickupAfterShot(highPriorityNote),
            autoIntakeTowardsNote(highPriorityNote),
            scoreRingAfterPickup(highPriorityNote).andThen(navigatePickupAfterShot(midPriorityNote)).onlyIf(() -> {return this.goodPickup;}),
            autoIntakeTowardsNote(midPriorityNote),
            scoreRingAfterPickup(midPriorityNote).andThen(navigatePickupAfterShot(lowPriorityNote)).onlyIf(() -> {return this.goodPickup;}),
            autoIntakeTowardsNote(lowPriorityNote),
            scoreRingAfterPickup(lowPriorityNote).onlyIf(() -> {return this.goodPickup;})
        ).withName(autoNamePrefix + " Source Side HyperChad Auto");
    }

    // private Command pathfindToNote(FieldElement note) {

    //     Pose2d targetPose = FlyingCircuitUtils.pickupAtNote(
    //             drivetrain.getPoseMeters().getTranslation(),
    //             note.getLocation().toTranslation2d(), 
    //             0.);


    //     return AutoBuilder.pathfindToPose(
    //             targetPose,
    //             DrivetrainConstants.pathfindingConstraints,
    //             0.
    //         );
    // }


    // private Command intakeAndThenShoot(Pose2d scoringPose) {
    //     //TODO: figure out how to navigate to nearest scoring location instead of a preset one
    //     //use lambdas or something for scoringPose
        
    //     return new SequentialCommandGroup(
    //         intakeTowardsNote(),
    //         new ParallelDeadlineGroup(
    //             AutoBuilder.pathfindToPose(scoringPose, DrivetrainConstants.pathfindingConstraints),
    //             prepAutoSpeakerShot()
    //         ),
    //         speakerShot()
    //     );
    // }

    // public Command pathfindingAuto(List<FieldElement> targetNotes, Pose2d scoringPose) {

        

    //     Command autoCommand = new InstantCommand();

    //     for (int noteInd = 0; noteInd < targetNotes.size(); noteInd++) {
    //         autoCommand = autoCommand
    //             .andThen(pathfindToNote(targetNotes.get(noteInd)).deadlineWith(resetShooter()))
    //             .andThen(intakeAndThenShoot(scoringPose).onlyIf(drivetrain::intakeSeesNote));
    //     }

    //     return autoCommand;
    // }



}
