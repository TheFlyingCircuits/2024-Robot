// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.FieldElement;
import frc.robot.commands.AimEverythingAtAmp;
import frc.robot.commands.NoteTrackingIndexNote;
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

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
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

    private Trigger ringJustEnteredIntake;
    private Trigger inSpeakerShotRange;
    
    
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
        
        NamedCommands.registerCommand("prepShot", prepAutoSpeakerShot());
        NamedCommands.registerCommand("shootFromAnywhere", speakerShot());
        NamedCommands.registerCommand("waitIndexNote", indexNote().withTimeout(4).finallyDo(() -> {drivetrain.isTrackingNote = false;}));
        NamedCommands.registerCommand("indexNote", indexNote().withTimeout(2).finallyDo(() -> {drivetrain.isTrackingNote = false;}));
        NamedCommands.registerCommand("intakeNote", intakeNote().withTimeout(1.5).finallyDo(() -> {drivetrain.isTrackingNote = false;}));
        NamedCommands.registerCommand("rapidFire", prepAutoSpeakerShot().alongWith(runIntake(true)));
        NamedCommands.registerCommand("trackNote", new InstantCommand(() -> {drivetrain.isTrackingNote = true;}));
        NamedCommands.registerCommand("resetShooter", resetShooter());
        NamedCommands.registerCommand("neverMissPickup", neverMissPickup());


        ringJustEnteredIntake = new Trigger(intake::ringJustEnteredIntake);
        inSpeakerShotRange = new Trigger(drivetrain::inSpeakerShotRange);

        ringJustEnteredIntake.onTrue(new InstantCommand(() -> {drivetrain.isTrackingNote = false;}));

        realBindings();
    }

    Command neverMissPickup() {
        return drivetrain.run(drivetrain::driveTowardsNote)
               .raceWith(intakeNote())
               .withTimeout(3.0);
    }

    /**
     * Aims the flywheels and arm at the speaker, while also setting the Pathplanner rotation override.
     * This command never finishes.
     */
    Command prepAutoSpeakerShot() {
        return new InstantCommand(() -> {drivetrain.isTrackingSpeakerInAuto = true;})
                .andThen(new PrepShot(false, drivetrain, arm, shooter, null, leds, FieldElement.SPEAKER))
                .finallyDo(() -> {drivetrain.isTrackingSpeakerInAuto = false;});

    }

    //these command compositions must be separated into their own methods
    //since wpilib requires a new instance of a command to be used in each
    //composition (the same instance of a command cannot be used in multiple compositions).

    /**
     * @param rapidFire - whether or not we are rapid firing (in auto); if we are, we want to run the indexer faster.
     */
    public Command runIntake(boolean rapidFire) {
        return new ScheduleCommand(leds.playIntakeAnimationCommand(drivetrain::shouldTrackNote))
               .andThen(
                    indexer.setOrangeWheelsSurfaceSpeedCommand(rapidFire ? 4 : 2.5)
                    .alongWith(intake.setVoltsCommand(12))
               );
    }

    public Command reverseIntake() {
        return indexer.run(() -> {indexer.setVolts(-8);})
               .alongWith(intake.setVoltsCommand(-8));
               // TODO: reverse LEDs when barfing?
    }

    /** Starts running the intake, and finishes when a note is detected by the bottom intake sensor.
     *  Intended to be followed up by indexNote, which will bring the note the rest of the way
     *  into the shooter until it hits the second intake/indexer sensor.
     *  Having intakeNote() be seperate from indexNote() will be usefull for auto,
     *  where we often want to start driving away from a note once we've acquired it,
     *  even if it isn't fully indexed yet.
     * @return
     */
    public Command intakeNote() {
        return this.runIntake(false).until(() -> {
            return intake.ringJustEnteredIntake() || indexer.isNoteIndexed();
        });
    }

    public Command indexNote() {
        return this.runIntake(false).until(indexer::isNoteIndexed)
               .andThen(new InstantCommand(() -> {indexer.setVolts(0); intake.setVolts(0);})) // TODO: remember why we need this.
               .andThen(new ScheduleCommand(leds.solidOrangeCommand()));
    }

    public Command signalNoteInIntake() {
        return new InstantCommand(() -> {
            // Find whatever pattern the LEDs are currently displaying,
            // so that we can return to that pattern after we're done with the strobe.
            Command interruptedPattern = leds.getCurrentCommand();
            interruptedPattern.equals(interruptedPattern);
            // TODO: check if the currently scheduled command is itself a strobe, so we don't get duplicate strobes?

            // Generate the strobe command
            Command strobe = leds.strobeCommand(Color.kWhite, 4, 0.5);

            // Make a command to re-schedule the original command.
            Command reSchedule = new ScheduleCommand(interruptedPattern);

            // Schedule the whole sequence.
            // Allow the strobe to happen while the robot is disabled for testing purposes.
            CommandScheduler.getInstance().schedule(strobe.andThen(reSchedule).ignoringDisable(true));
        });
    }

    /** Resets the angle and speed of the shooter back to its default idle position. */
    Command resetShooter() {
        double desiredAngle = ArmConstants.armMinAngleDegrees+5; // puts the arm at min height to pass under stage
        //desiredAngle = 35;
        return arm.setDesiredDegreesCommand(desiredAngle)
               .alongWith(shooter.setFlywheelSurfaceSpeedCommand(0));
               //.alongWith(indexer.setIndexerRPMCommand(0)));
    }

    /** Moves the arm back and spins up the flywheels to prepare for an amp shot. */
    Command prepAmpShot() {
        return new AimEverythingAtAmp(false, drivetrain, arm, shooter, charlie::getRequestedFieldOrientedVelocity, leds);
    }

    /** Moves the arm back and spins up the flywheels to prepare for a trap shot. */
    Command prepTrapShot() {
        return arm.setDesiredDegreesCommand(90)
               .alongWith(shooter.setFlywheelSurfaceSpeedCommand(10));
    }

    Command prepLobShot() {
        return new PrepShot(true, drivetrain, arm, shooter, charlie::getRequestedFieldOrientedVelocity, leds, FieldElement.LOB_TARGET);
    }

    Command shootAtTarget(FieldElement target) {
        PrepShot aim = new PrepShot(true, drivetrain, arm, shooter, charlie::getRequestedFieldOrientedVelocity, leds, target);
        Command waitForAlignment = new WaitUntilCommand(aim::readyToShoot);
        Command fire = fireNote();
        if (target == FieldElement.SPEAKER || target == FieldElement.LOB_TARGET) {
            fire = fire.withTimeout(0.2);
        }
        return aim.raceWith(waitForAlignment.andThen(fireNote()));
    }

    Command shart() {
        return shootAtTarget(FieldElement.RIGHT_IN_FRONT_OF_YOU);
    }

    Command lobShot() {
        return shootAtTarget(FieldElement.LOB_TARGET);
    }

    public Command speakerShot() {
        return shootAtTarget(FieldElement.SPEAKER);
    }

    public Command fireNote() {
        return indexer.setOrangeWheelsSurfaceSpeedCommand(7).withTimeout(0.4)
               .alongWith(new ScheduleCommand(leds.playFireNoteAnimationCommand()));
    }


    private void realBindings() {
        CommandXboxController controller = charlie.getXboxController();
        /** INTAKE **/
        controller.rightTrigger()
            //.onTrue(intakeNote().raceWith(resetShooter()));
            //.whileTrue(new NoteTrackingIndexNote(intake, indexer, drivetrain, charlie::getRequestedFieldOrientedVelocity));
            .onTrue(indexNote().raceWith(resetShooter())); // reset never ends, indexNote does.
    
        controller.leftTrigger().whileTrue(reverseIntake());
        
        
        /** SCORING **/
        
        //speaker shot
        controller.rightBumper().and(inSpeakerShotRange)
            .onTrue(
                this.speakerShot()
                .andThen(new ScheduleCommand(this.resetShooter())));

        //lob shot (prep while holding, release to fire)
        controller.rightBumper().and(inSpeakerShotRange.negate())
            .onTrue(prepLobShot())
            .onFalse(this.fireNote()
                .andThen(new ScheduleCommand(this.resetShooter()))
                .unless(() -> arm.getDegrees() < 15));
            



        
        // controller.rightBumper().onTrue(
        //     new ConditionalCommand(
        //         this.speakerShot(), 
        //         this.lobShot(),
        //         drivetrain::inSpeakerShotRange)
        //     .andThen(new ScheduleCommand(this.resetShooter())));

        //.onTrue(this.speakerShot().andThen(new ScheduleCommand(this.resetShooter())));
        //.onTrue(this.lobShot().andThen(new ScheduleCommand(this.resetShooter())));
        //.onTrue(prepAutoSpeakerShot().alongWith(runIntake()));

        controller.leftBumper()
            .onTrue(prepAmpShot())
            .onFalse(this.fireNote().andThen(new ScheduleCommand(this.resetShooter())));
            // use a schedule command so the onFalse sequence doesn't cancel the aiming while the note is being shot.
            
        controller.b().onTrue(shart().andThen(new ScheduleCommand(this.resetShooter())));

        /** CLIMB **/
        
        controller.povUp().onTrue(climb.raiseHooksCommand());
        controller.povDown().onTrue(climb.lowerHooksCommand().until(climb::climbArmsZero));
        controller.a().whileTrue(new TrapRoutine(charlie::getRequestedFieldOrientedVelocity, climb, arm, shooter, indexer, leds, drivetrain));

        /** MISC **/
        controller.y().onTrue(new InstantCommand(() -> drivetrain.setPoseToVisionMeasurement()));
        ben.y().onTrue(new InstantCommand(() -> drivetrain.setPoseToVisionMeasurement()));
        //controller.y().onTrue(new InstantCommand(() -> drivetrain.setRobotFacingForward()));

        controller.x().onTrue(new InstantCommand(() -> arm.setDisableSetpointChecking(false)).andThen(resetShooter()));

        /** Driver Feedback **/
        ringJustEnteredIntake.onTrue(charlie.rumbleController(0.25, 0.5)); // lol this happens even during auto
        ringJustEnteredIntake.onTrue(this.signalNoteInIntake().ignoringDisable(true));
        // TODO: prevent flash on reverse? Either condition with positive wheel speeds,
        //       or no seperate scheudle command?
    }
}
