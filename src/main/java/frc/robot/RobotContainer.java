// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ArmConstants;
import frc.robot.commands.arm.AimShooterAtAngle;
import frc.robot.commands.arm.AimShooterAtSpeaker;
import frc.robot.commands.climb.LowerClimbArms;
import frc.robot.commands.climb.RaiseClimbArms;
import frc.robot.commands.drivetrain.AimAtSpeakerWhileJoystickDrive;
import frc.robot.commands.drivetrain.JoystickDrive;
import frc.robot.commands.intake.IndexNote;
import frc.robot.commands.intake.IntakeNote;
import frc.robot.commands.intake.ReverseIntake;
import frc.robot.commands.leds.ChasePattern;
import frc.robot.commands.leds.CheckerboardGreen;
import frc.robot.commands.leds.ShooterChargeUp;
import frc.robot.commands.leds.SolidBlue;
import frc.robot.commands.leds.SolidOrange;
import frc.robot.commands.shooter.FireNote;
import frc.robot.commands.shooter.SpinFlywheels;
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
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOKraken;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonLib;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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

    public final static CommandXboxController controller = new CommandXboxController(0);
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
                new SwerveModuleIOKraken(3, 1, -0.00342, 7, false, false),
                new SwerveModuleIOKraken(2, 2, 0.36816, 6, false, false),
                new SwerveModuleIOKraken(1, 5, -0.09009, 5, false, true),
                new SwerveModuleIOKraken(0, 6, -0.37622, 4, true, false),
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

        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        Logger.start();
        
        
        drivetrain.setDefaultCommand(new JoystickDrive(true, drivetrain));

        isRingInIntake = new Trigger(intake::isRingInIntake);
        
        
        NamedCommands.registerCommand("shootFromSubwoofer", shootFromSubwoofer());
        NamedCommands.registerCommand("shootFromAnywhere", shootFromAnywhere());
        NamedCommands.registerCommand("indexNote", indexNote());
        NamedCommands.registerCommand("indexWithTimeout", indexWithTimeout(1.0));


        realBindings();
    }

    /** Generates a command to rumble the controller for a given duration and strength.
     * @param seconds - Time to rumble the controller for, in seconds.
     * @param strength - Strength to rumble the controller at, from 0 to 1.
     */
    Command rumbleController(double seconds, double strength) {
        return new InstantCommand(() -> controller.getHID().setRumble(RumbleType.kBothRumble, strength))
            .andThen(new WaitCommand(seconds))
            .andThen(new InstantCommand(() -> controller.getHID().setRumble(RumbleType.kBothRumble, 0)));
    }


    //these command compositions must be separated into their own methods
    //since wpilib requires a new instance of a command to be used in each
    //composition (the same instance of a command cannot be used in multiple compositions).

    Command intakeNote() {
        return new ParallelRaceGroup(
            new IntakeNote(intake),
            new SolidOrange(leds)
        );
    }

    Command indexNote() {
        return new ParallelRaceGroup(
            new IndexNote(intake, indexer),
            new ChasePattern(leds)
        );
    }

    //has a timeout after a set time indexing, is used during auto
    //so that we keep moving if we miss a note
    Command indexWithTimeout(double seconds) {
        return new IndexNote(intake, indexer).withTimeout(seconds);
    }

    Command aimShooterAtAngle(double angle) {
        return new ParallelRaceGroup(
            new AimShooterAtAngle(angle, arm),
            new SolidBlue(leds));
    }

    Command spinFlywheels(double leftFlywheelMetersPerSecond, double rightFlywheelsMetersPerSecond) {
        return new ParallelRaceGroup(
            new SpinFlywheels(leftFlywheelMetersPerSecond, leftFlywheelMetersPerSecond, shooter),
            new ShooterChargeUp(leds, shooter, leftFlywheelMetersPerSecond)
        );
    }



    /** Resets the angle and speed of the shooter back to its default idle position. */
    Command resetShooter() {
        return new ParallelCommandGroup(
            spinFlywheels(0, 0),
            aimShooterAtAngle(ArmConstants.armMinAngleDegrees+5));
    }

    /** Moves the arm back and spins up the flywheels to prepare for an amp shot. */
    Command prepAmpShot() {
        return new ParallelCommandGroup(
            spinFlywheels(15, 15),
            aimShooterAtAngle(110));
    }

    /** Moves the arm back and spins up the flywheels to prepare for a trap shot. */
    Command prepTrapShot() {
        return new ParallelCommandGroup(
            spinFlywheels(15, 15),
            aimShooterAtAngle(90));
    }

    /** Spins the flywheels up to speed and aims arm when pressed against the subwoofer. */
    Command prepSubwooferShot() {
        return new ParallelCommandGroup(
            spinFlywheels(27, 15),
            aimShooterAtAngle(45));
    }

    Command prepShart() { 
        return new ParallelCommandGroup(
            spinFlywheels(25, 25),
            aimShooterAtAngle(-15));
    }

    
    /** Command to prepare for a shot. Finishes after the flywheels spin up to a desired
     *  speed, and the shooter reaches the correct angle. */
    Command prepShotFromAnywhere() { 
        return new ParallelRaceGroup(
            new ParallelCommandGroup(
                spinFlywheels(27, 15),
                new AimShooterAtSpeaker(arm, drivetrain)),
            new AimAtSpeakerWhileJoystickDrive(drivetrain));
    }


    Command fireNote() {
        return new ParallelRaceGroup(
            new FireNote(indexer),
            new CheckerboardGreen(leds)
        );
    }

    //aims and then shoots in one motion
    Command shootFromSubwoofer() { 
        return new SequentialCommandGroup(
            prepSubwooferShot(),
            fireNote(),
            resetShooter());
    }


    //aims and then shoots in one motion
    SequentialCommandGroup shootFromAnywhere() {
        return new SequentialCommandGroup(
            prepShotFromAnywhere(),
            fireNote(),
            resetShooter());
    }

    
    Command shart() {
        return new SequentialCommandGroup(
            prepShart(),
            fireNote(),
            resetShooter()
        );
    }

    private void realBindings() {
        /** INTAKE **/
        controller.rightTrigger()
            .whileTrue(intakeNote())
            .onFalse(indexNote());
    
        controller.leftTrigger().whileTrue(new ReverseIntake(intake, indexer));
        
        
        /** SCORING **/
        controller.rightBumper().onTrue(shootFromAnywhere());
        controller.leftBumper()
            .whileTrue(prepAmpShot())
            .onFalse(fireNote().andThen(resetShooter()));
        controller.b().whileTrue(shart());


        /** CLIMB **/
        //climb routine should be tilt shooter back, drive chain over shooter arm, raise arm to amp shot, climb, score trap
        //in other words, press up then left then right then down and then LB
        controller.povUp().onTrue(new RaiseClimbArms(climb).alongWith(aimShooterAtAngle(ArmConstants.armMaxAngleDegrees)));
        controller.povRight().onTrue(aimShooterAtAngle(90));
        controller.povDown().whileTrue(new LowerClimbArms(climb).alongWith(prepTrapShot()));
        controller.povLeft().onTrue(fireNote().andThen(new InstantCommand(() -> spinFlywheels(0, 0))));

        /** MISC **/
        controller.y().onTrue(new InstantCommand(() -> drivetrain.setRobotFacingForward()));

        controller.x().onTrue(resetShooter());

        isRingInIntake.onTrue(rumbleController(0.25, 0.5));
    }

    private void testBindings() {
        /** INTAKE **/
        controller.rightTrigger()
            .whileTrue(intakeNote())
            .onFalse(indexNote());
    
        controller.leftTrigger().whileTrue(new ReverseIntake(intake, indexer));
        
        
        /** SCORING **/
        // controller.rightBumper().onTrue(shootFromAnywhere());
        // controller.leftBumper()
        //     .whileTrue(prepAmpShot())
        //     .onFalse(fireNote().andThen(resetShooter()));
        // controller.b().whileTrue(shart());

        controller.povUp().whileTrue(new RaiseClimbArms(climb));
        controller.povDown().whileTrue(new LowerClimbArms(climb));
        controller.povLeft().onTrue(aimShooterAtAngle(ArmConstants.armMaxAngleDegrees));
        controller.povRight().onTrue(prepTrapShot());
        controller.a().onTrue(fireNote().andThen(new InstantCommand(() -> spinFlywheels(0, 0))));

        /** MISC **/
        controller.y().onTrue(new InstantCommand(() -> drivetrain.setRobotFacingForward()));

        controller.x().onTrue(resetShooter());
        
        controller.leftStick().onTrue(new InstantCommand(() -> arm.setArmDesiredPosition(ArmConstants.armMinAngleDegrees)));

        isRingInIntake.onTrue(rumbleController(0.25, 0.5));

        /** SYSID BINDINGS **/
        // controller.a().whileTrue(arm.generateSysIdQuasistatic(SysIdRoutine.Direction.kForward));
        // controller.b().whileTrue(arm.generateSysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        
        // controller.x().whileTrue(arm.generateSysIdDynamic(SysIdRoutine.Direction.kForward));
        // controller.y().whileTrue(arm.generateSysIdDynamic(SysIdRoutine.Direction.kReverse));
    
    }
}
