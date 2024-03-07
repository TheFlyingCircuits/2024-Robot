// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.commands.AimEverythingAtSpeaker;
import frc.robot.commands.intake.IndexNote;
import frc.robot.commands.intake.ReverseIntake;
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
import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
    public final Shooter shooter;
    public final Drivetrain drivetrain;
    public final Arm arm;
    public final Intake intake;
    public final Indexer indexer;
    public final Climb climb;
    public final LEDs leds;

    private Trigger isRingInIntake;
    
    
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

        
        /**** ADVANTAGE KIT LOGGER  *****/
        Logger.recordMetadata("projectName", "2024Robot");

        Logger.addDataReceiver(new NT4Publisher());
        Logger.start();
        
        
        drivetrain.setDefaultCommand(drivetrain.fieldOrientedDriveCommand(charlie::getRequestedFieldOrientedVelocity));
        leds.setDefaultCommand(leds.heartbeatCommand().ignoringDisable(true));
        intake.setDefaultCommand(Commands.run(() -> {intake.setVolts(0);}, intake));
        indexer.setDefaultCommand(indexer.setIndexerRPMCommand(0));
        shooter.setDefaultCommand(shooter.setFlywheelSurfaceSpeedCommand(0));
        climb.setDefaultCommand(Commands.run(() -> {climb.setVolts(0);}, climb));
        arm.setDefaultCommand(arm.holdCurrentPositionCommand().ignoringDisable(true));

        isRingInIntake = new Trigger(intake::isRingInIntake);
        
        //NamedCommands.registerCommand("shootFromAnywhere", shootFromAnywhereNoReset());
        NamedCommands.registerCommand("indexNote", indexNote().alongWith(resetShooter()));
        //NamedCommands.registerCommand("continuousPrepShotFromAnywhere", continuousPrepShotFromAnywhereNoDrivetrain());
        NamedCommands.registerCommand("trackNote", new InstantCommand(() -> drivetrain.isTrackingNote = true));
        isRingInIntake.onTrue(new InstantCommand(() -> drivetrain.isTrackingNote = false));

        configAutoBuilder();

        realBindings();
    }


    private void configAutoBuilder() {
        AutoBuilder.configureHolonomic(
            drivetrain::getPoseMeters, // Robot pose supplier
            drivetrain::setPoseMeters, // Method to reset odometry (will be called if your auto has a starting pose)
            drivetrain::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (ChassisSpeeds speeds) -> drivetrain.robotOrientedDrive(speeds, true), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(4.0, 0.0, 0.0), // Rotation PID constants
                    DrivetrainConstants.maxAchievableVelocityMetersPerSecond, // Max module speed, in m/s
                    DrivetrainConstants.drivetrainRadiusMeters, // Drive base radius in meters. Distance from robot center to furthest module.
                    new ReplanningConfig() // Default path replanning config. See the API for the options here
            ),
            () -> {
              // Boolean supplier that controls when the path will be mirrored
              // We by default draw the paths on the red side of the field, mirroring them if we are on the blue alliance.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Blue;
              }
              return false;
            },
            drivetrain // Reference to this subsystem to set requirements
        );

        PPHolonomicDriveController.setRotationTargetOverride(drivetrain::getAutoRotationOverride);
    }


    /** Generates a command to rumble the controller for a given duration and strength.
     * @param seconds - Time to rumble the controller for, in seconds.
     * @param strength - Strength to rumble the controller at, from 0 to 1.
     */
    Command rumbleController(double seconds, double strength) {
        return new InstantCommand(() -> charlie.getXboxController().getHID().setRumble(RumbleType.kBothRumble, strength))
            .andThen(new WaitCommand(seconds))
            .andThen(new InstantCommand(() -> charlie.getXboxController().getHID().setRumble(RumbleType.kBothRumble, 0)));
    }


    //these command compositions must be separated into their own methods
    //since wpilib requires a new instance of a command to be used in each
    //composition (the same instance of a command cannot be used in multiple compositions).
    public Command indexNote() {
        return new ScheduleCommand(leds.playIntakeAnimationCommand())
               .andThen(new IndexNote(intake, indexer))
               .andThen(new ScheduleCommand(leds.solidOrangeCommand()));
    }

    /** Resets the angle and speed of the shooter back to its default idle position. */
    Command resetShooter() {
        return arm.setDesiredDegreesCommand(ArmConstants.armMinAngleDegrees+5)
               .alongWith(shooter.setFlywheelSurfaceSpeedCommand(0));
    }

    /** Moves the arm back and spins up the flywheels to prepare for an amp shot. */
    Command prepAmpShot() {
        return arm.setDesiredDegreesCommand(110)
               .alongWith(shooter.setFlywheelSurfaceSpeedCommand(10));
    }

    /** Moves the arm back and spins up the flywheels to prepare for a trap shot. */
    Command prepTrapShot() {
        return arm.setDesiredDegreesCommand(90)
               .alongWith(shooter.setFlywheelSurfaceSpeedCommand(20));
    }

    /** Spins the flywheels up to speed and aims arm when pressed against the subwoofer. */
    Command prepSubwooferShot() {
        return arm.setDesiredDegreesCommand(42)
               .alongWith(shooter.setFlywheelSurfaceSpeedCommand(27));
    }

    Command prepShart() {
        return arm.setDesiredDegreesCommand(-15)
               .alongWith(shooter.setFlywheelSurfaceSpeedCommand(25));
    }
    
    // /** Command to prepare for a shot. Finishes after the flywheels spin up to a desired
    //  *  speed, the shooter reaches the correct angle, and the drivetrain aims at the right
    //  *  angle.*/
    // Command prepShotFromAnywhere() { 
    //     return new ParallelCommandGroup(
    //         new SpinFlywheels(27, shooter),
    //         new AimShooterAtSpeaker(arm, drivetrain),
    //         new AimDriveAtSpeaker(drivetrain));
    // }

    // /** Command to prepare for a shot. Same as prepShotFromAnywhere(), but never finishes
    //  *  until interrupted. This allows us to prepare our shot while we're still moving.
    //  *  This command also doesn't rotate the drivetrain at all.
    //  */
    // Command continuousPrepShotFromAnywhereNoDrivetrain() {
    //     return new ParallelCommandGroup(
    //         new SpinFlywheels(27, shooter),
    //         new ContinuousAimShooterAtSpeaker(arm, drivetrain));
    // }

    // /**
    //  * Command to prepare for a shot. Same as prepShotFromAnywhere(), but never finishes until
    //  * interrupted. This allows us to prepare our shot while we're still moving.
    //  */
    // Command continuousPrepShotFromAnywhereWithDrivetrain() {
    //     return new ParallelCommandGroup(
    //         continuousPrepShotFromAnywhereNoDrivetrain(),
    //         new AimAtSpeakerWhileJoystickDrive(drivetrain)
    //     );
    // }

    //aims and then shoots in one motion
    Command shootFromSubwoofer() {
        // TODO: check progress for prepSubwooferShot()
        return prepSubwooferShot().andThen(indexer.fireNoteCommand()).andThen(resetShooter());
    }


    // /** Aims and shoots in one motion, and then resets the shooter back to idle mode. */
    // SequentialCommandGroup shootFromAnywhere() {
    //     return new SequentialCommandGroup(
    //         prepShotFromAnywhere(),
    //         fireNote(),
    //         resetShooter());
    // }

    // /** Aims and shoots in one motion, but arm remains up and spinning at the end. */
    // SequentialCommandGroup shootFromAnywhereNoReset() {
    //     return new SequentialCommandGroup(
    //         prepShotFromAnywhere(),
    //         fireNote());
    // }

    
    
    Command shart() {
        // TODO: check progress for prepShart()
        return prepShart().andThen(indexer.fireNoteCommand()).andThen(resetShooter());
    }

    private void realBindings() {
        CommandXboxController controller = charlie.getXboxController();
        /** INTAKE **/
        controller.rightTrigger()
            .onTrue(indexNote().alongWith(resetShooter()));
    
        controller.leftTrigger().whileTrue(new ReverseIntake(intake, indexer));
        
        
        /** SCORING **/
        //control scheme is rb/lb toggle preps a shot, and then a is fire
        controller.rightBumper()
            //.onTrue(continuousPrepShotFromAnywhereWithDrivetrain())
            //.onFalse(resetShooter());
            .whileTrue(new AimEverythingAtSpeaker(drivetrain, arm, shooter, charlie::getRequestedFieldOrientedVelocity, leds))
            .onFalse(resetShooter());
        controller.leftBumper()
            .onTrue(prepAmpShot())
            .onFalse(resetShooter());
        //controller.b().whileTrue(shart());
        controller.b().onTrue(prepShart())
                      .onFalse(resetShooter());
        controller.a().onTrue(indexer.fireNoteCommand().alongWith(leds.playFireNoteAnimationCommand()));


        /** CLIMB **/
        //climb routine should be tilt shooter back, drive chain over shooter arm, raise arm to amp shot, climb, score trap
        //in other words, press up then left then right then down and then LB
        controller.povUp().onTrue(climb.raiseHooksCommand().alongWith(arm.setDesiredDegreesCommand(ArmConstants.armMaxAngleDegrees)));
        controller.povRight().onTrue(arm.setDesiredDegreesCommand(82));
        controller.povDown().whileTrue(climb.lowerHooksCommand().alongWith(prepTrapShot()));
        controller.start().onTrue(indexer.fireNoteCommand().andThen(shooter.setFlywheelSurfaceSpeedCommand(0)));

        /** MISC **/
        controller.y().onTrue(new InstantCommand(() -> drivetrain.setRobotFacingForward()));

        controller.x().onTrue(resetShooter());

        isRingInIntake.onTrue(charlie.rumbleController(0.25, 0.5));
        isRingInIntake.onTrue(leds.strobeCommand(Color.kWhite, 4, 0.5).ignoringDisable(true));//.andThen(new ScheduleCommand(leds.playIntakeAnimationCommand())));
        // TODO: prevent flash on reverse? Either condition with positive wheel speeds,
        //       or no seperate scheudle command?
    }

    private void testBindings() {
        CommandXboxController controller = charlie.getXboxController();

        controller.rightTrigger().whileTrue(indexNote());
        controller.leftTrigger().whileTrue(new ReverseIntake(intake, indexer));

        // controller.povRight().onTrue(aimShooterAtAngle(0));
        // controller.povUp().onTrue(aimShooterAtAngle(20));
        // controller.povLeft().onTrue(aimShooterAtAngle(30));
        // controller.povDown().onTrue(aimShooterAtAngle(45));

        /** SYSID BINDINGS **/
        // controller.a().whileTrue(arm.generateSysIdQuasistatic(SysIdRoutine.Direction.kForward));
        // controller.b().whileTrue(arm.generateSysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        
        // controller.x().whileTrue(arm.generateSysIdDynamic(SysIdRoutine.Direction.kForward));
        // controller.y().whileTrue(arm.generateSysIdDynamic(SysIdRoutine.Direction.kReverse));
    
    }
}
