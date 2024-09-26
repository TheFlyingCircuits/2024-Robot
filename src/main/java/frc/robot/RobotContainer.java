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
import edu.wpi.first.wpilibj.Timer;
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
        leds.setDefaultCommand(leds.heartbeatCommand().ignoringDisable(true).withName("heartbeat"));
        // leds.setDefaultCommand(leds.heartbeatCommand(1.5).andThen(leds.heartbeatCommand(1.0)).ignoringDisable(true));
        // leds.setDefaultCommand(leds.fasterHeartbeatSequence().ignoringDisable(true));
        intake.setDefaultCommand(Commands.run(() -> {intake.setVolts(0);}, intake));
        indexer.setDefaultCommand(indexer.setBlackRollerSurfaceSpeedCommand(0));
        shooter.setDefaultCommand(shooter.setFlywheelSurfaceSpeedCommand(0));
        climb.setDefaultCommand(Commands.run(() -> {climb.setVolts(0);}, climb));
        arm.setDefaultCommand(arm.holdCurrentPositionCommand().ignoringDisable(true));

        dontAmpAutoAlign = ben.leftBumper();


        realBindings();

        SmartDashboard.putData(leds);
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

                intakeNote().until(indexer::isNoteIndexed)
                    .andThen(new ScheduleCommand(
                        indexNote().andThen(reverseIntake().withTimeout(1.0))
                    ))
            );



            //.onTrue(intakeNote().andThen(indexNote()));
            //.onTrue(indexNote().raceWith(resetShooter())); // reset never ends, indexNote does.
    
        controller.leftTrigger().whileTrue(reverseIntake().alongWith(indexer.run(() -> {indexer.setVolts(-8);})));
        
        
        /** SCORING **/
        //speaker shot
        // Trigger inSpeakerShotRange = new Trigger(drivetrain::inSpeakerShotRange);
        // controller.rightBumper().and(inSpeakerShotRange)
        //     .onTrue(
        //         this.speakerShot()
        //         .andThen(new ScheduleCommand(this.resetShooter()))
        //     );

        // //lob shot (prep while holding, release to fire)
        // Command prepLobShot = prepLobShot(); // <- grab a single instance so we can check if it's cancelled.
        // controller.rightBumper().and(inSpeakerShotRange.negate())
        //     .onTrue(prepLobShot)
        //     .onFalse(this.fireNote()
        //                  .andThen(new ScheduleCommand(this.resetShooter()))
        //              .unless(() -> {return !prepLobShot.isScheduled();})
        //              // only fire if the lob shot wasn't cancelled
        //     );


        //hold to prep, release to fire
        controller.rightBumper()
            .onTrue(
                this.ringShot()
                    .andThen(new ScheduleCommand(this.resetShooter()))
            );
            
        // controller.leftBumper()
        //     .onTrue(prepAmpShot())
        //     .onFalse(this.fireNoteThroughHood().andThen(new ScheduleCommand(this.resetShooter())));
            // use a schedule command so the onFalse sequence doesn't cancel the aiming while the note is being shot.
            
        // controller.b().onTrue(shart().andThen(new ScheduleCommand(this.resetShooter())));

        /** CLIMB **/
        
        // controller.povUp().onTrue(climb.raiseHooksCommand());
        // controller.povRight().onTrue(climb.homeHooksCommand());
        // controller.povDown().onTrue(climb.lowerHooksCommand().until(climb::atQuickClimbSetpoint).withTimeout(3));

        // controller.a()
        //         .onTrue(new InstantCommand(() -> {drivetrain.onlyUseTrapCamera = true;}))
        //         .whileTrue(
        //             new UnderStageTrapRoutine(
        //                 charlie::getRequestedFieldOrientedVelocity, climb, arm, shooter, drivetrain))
        //         .onFalse(new InstantCommand(() -> {drivetrain.onlyUseTrapCamera = false;}));

        // //manual fire and then lower, so we aren't hanging off of trap edge
        // controller.back()
        //     .whileTrue(fireNoteThroughHood().repeatedly())
        //     .onFalse(new ScheduleCommand(climb.raiseHooksCommand(4).withTimeout(0.5))); //this cancels the trap routine

        


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
        // controller.y().onTrue(new InstantCommand(() -> drivetrain.setPoseToVisionMeasurement()).repeatedly().until(drivetrain::seesTag));
        // ben.y().onTrue(new InstantCommand(() -> drivetrain.setPoseToVisionMeasurement()).repeatedly().until(drivetrain::seesTag));
        controller.y().onTrue(new InstantCommand(() -> drivetrain.setRobotFacingForward()));
        controller.x().onTrue(new InstantCommand(() -> arm.setDisableSetpointChecking(false)).andThen(resetShooter()));

        // controller.start().whileTrue(new MeasureWheelDiameter(drivetrain));

        /** Driver Feedback **/
        Trigger ringJustEnteredIntake = new Trigger(intake::ringJustEnteredIntake);
        ringJustEnteredIntake.onTrue(charlie.rumbleController(0.5).withTimeout(0.25)); // lol this happens even during auto
        ringJustEnteredIntake.onTrue(leds.temporarilySwitchPattern(() -> {return leds.strobeCommand(Color.kWhite, 4, 0.5).ignoringDisable(true);}).ignoringDisable(true));
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
        return new ScheduleCommand(leds.playIntakeAnimationCommand(() -> {return drivetrain.getBestNoteLocationFieldFrame().isPresent();}).withName("intake animation"))
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
        return this.runIntake(true).until(indexer::isNoteIndexed).unless(indexer::isNoteIndexed)
               .andThen(new InstantCommand(() -> {indexer.setVolts(0); intake.setVolts(0);})
                        .alongWith(new ScheduleCommand(leds.solidOrangeCommand()))
               );
    }
    
        
    private Command fireNote() {
        return indexer.setOrangeWheelsSurfaceSpeedCommand(7).withTimeout(0.25)
               .alongWith(new ScheduleCommand(leds.playFireNoteAnimationCommand()));
    }

    private Command fireNoteThroughHood() {
        // The flywheels need to run longer than normal to make sure the note
        // makes it all the way through the slam dunk hood.
        return indexer.setOrangeWheelsSurfaceSpeedCommand(7).withTimeout(1.0)
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

    //for activity fair
    private Command prepRingShot() {
        return new PrepShot(drivetrain, arm, shooter, charlie::getRequestedFieldOrientedVelocity, leds, FieldElement.POLE);
        
    }

    //for centennial
    private Command ringShot() {
        PrepShot aim = new PrepShot(drivetrain, arm, shooter, charlie::getRequestedFieldOrientedVelocity, leds, FieldElement.POLE);
        Command waitForAlignment = new WaitUntilCommand(aim::readyToShoot);
        return aim.raceWith(waitForAlignment.andThen(fireNote()));
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
            .until(() -> {return noteIsLostCauseInAuto(note);}).withTimeout(4.0)
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

        // if the first note is too far away, then that's the note we'll likely see on the next frame
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
     * @param note - the note you want to pickup.
     * @param startLocation 
     */
    private Command navigatePickup(FieldElement note, String startLocation) {
        char noteNuber = note.name().charAt(note.name().length()-1);
        String pathName = startLocation+" to Ring "+noteNuber;

        return new ParallelDeadlineGroup(
                FlyingCircuitUtils.followPath(pathName),
                resetShooter(),
                new PrintCommand("Driving from "+startLocation+" to "+ note.name())
        );
    }

    /**
     * Navigates from a note to a scoring location after a pickup.
     * @param note - the note you are navigating from.
     */
    private Command scoreRingAfterPickup(FieldElement note, String endLocation) {
        char noteNuber = note.name().charAt(note.name().length()-1);
        String pathName = "Ring "+noteNuber+" to "+endLocation;

        // Don't aim on the way if we're going under the stage
        boolean aimOnTheWay = !endLocation.contains("Center Side");

        return new SequentialCommandGroup(
            new ParallelDeadlineGroup(
                FlyingCircuitUtils.followPath(pathName),
                new ConditionalCommand(
                    indexNote().andThen(prepAutoSpeakerShot()), 
                    indexNote(),
                    () -> {return aimOnTheWay;}),
                new PrintCommand("Scoring " + note.name())
            ),
            speakerShot()
        );
    }


    public Command centerSideAuto(FieldElement noteToGoFor) {
        char noteNumber = noteToGoFor.name().charAt(noteToGoFor.name().length()-1);
        String outgoingPathName = "Starting Line (Center Side) to Ring "+noteNumber;
        return new SequentialCommandGroup(
            // Fire preload
            // speakerShot(),

            // Pickup ring 6
            new ParallelDeadlineGroup(
                // navigatePickup(FieldElement.NOTE_6, "Starting Line (Center Side)"),
                FlyingCircuitUtils.followPath(outgoingPathName),
                new SequentialCommandGroup(
                    new ParallelDeadlineGroup(
                        new WaitUntilCommand(() -> {
                            boolean armGoodEnough = arm.getDegrees() >= 35;
                            return armGoodEnough && (drivetrain.getAngleError() <= 2);
                        }).andThen(fireNote()),
                        prepAutoSpeakerShot()
                    ),
                    resetShooter()
                )
            ),
            autoIntakeTowardsNote(noteToGoFor),

            // Return trip and shot
            scoreRingAfterPickup(noteToGoFor, "Starting Line (Center Side)"),

            // Start rapid fire sequence
            new ParallelRaceGroup(
                // Run intake while aiming
                prepAutoSpeakerShot().alongWith(runIntake(false)),

                // Follow the rapid fire path
                new SequentialCommandGroup(
                    FlyingCircuitUtils.followPath("Ring 3 Rapid Pickup"),
                    FlyingCircuitUtils.followPath("Ring 2 Rapid Pickup"),
                    FlyingCircuitUtils.followPath("Ring 1 Rapid Pickup"),
                    new WaitCommand(1.0) // Let the flywheels keep running for a sec even if the path ends.
                )
            )

        ).withName("("+noteNumber+", 3, 2, 1) 5 Piece Center Side");
    }

    public Command sideAuto(FieldElement[] notesToGoFor, FieldElement startLocation) {
        FieldElement highPriorityNote = notesToGoFor[0];
        FieldElement midPriorityNote = notesToGoFor[1];
        FieldElement lowPriorityNote = notesToGoFor[2];

        String side = "";
        if (startLocation == FieldElement.AMP) {
            side = "Amp";
        }
        if (startLocation == FieldElement.SOURCE) {
            side = "Source";
        }
        String shotName = side+" Shot";

        Command auto = new SequentialCommandGroup(
            speakerShot(),
            navigatePickup(highPriorityNote, "Starting Line ("+side+" Side)"),
            autoIntakeTowardsNote(highPriorityNote),
            scoreRingAfterPickup(highPriorityNote, shotName).andThen(navigatePickup(midPriorityNote, shotName)).onlyIf(() -> {return this.goodPickup;}),
            autoIntakeTowardsNote(midPriorityNote),
            scoreRingAfterPickup(midPriorityNote, shotName).andThen(navigatePickup(lowPriorityNote, shotName)).onlyIf(() -> {return this.goodPickup;}),
            autoIntakeTowardsNote(lowPriorityNote),
            scoreRingAfterPickup(lowPriorityNote, shotName).onlyIf(() -> {return this.goodPickup;})
        );

        // Name the auto so it looks nice on the dashboard.
        char highPriorityAsChar = highPriorityNote.name().charAt(highPriorityNote.name().length()-1);
        char midPriorityAsChar = midPriorityNote.name().charAt(midPriorityNote.name().length()-1);
        char lowPriorityAsChar = lowPriorityNote.name().charAt(lowPriorityNote.name().length()-1);
        String autoName = "("+highPriorityAsChar+", "+midPriorityAsChar+", "+lowPriorityAsChar+") HyperChad "+side+" Side";
        return auto.withName(autoName);
    }




}
