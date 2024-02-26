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
import frc.robot.commands.leds.SolidBlue;
import frc.robot.commands.leds.SolidOrange;
import frc.robot.commands.shooter.FireNote;
import frc.robot.commands.shooter.SpinFlywheels;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIO;
import frc.robot.subsystems.arm.ArmIONeo;
import frc.robot.subsystems.arm.ArmIOSim;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.GyroIOPigeon;
import frc.robot.subsystems.drivetrain.GyroIOSim;
import frc.robot.subsystems.drivetrain.SwerveModuleIOKraken;
import frc.robot.subsystems.drivetrain.SwerveModuleIONeo;
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

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic shoud actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instlead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

    public static final CommandXboxController controller = new CommandXboxController(0);
    public final Shooter shooter;
    public final Drivetrain drivetrain;
    public final Arm arm;
    public final Intake intake;
    public final Indexer indexer;
    public final Climb climb;
    public final LEDs leds;

    private Trigger isRingInIntake;
    
    public RobotContainer() {

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
        
        
        drivetrain.setDefaultCommand(new JoystickDrive(true, drivetrain));

        isRingInIntake = new Trigger(intake::isRingInIntake);
        
        testBindings();
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

    Command aimShooterAtAngle(double angle) {
        return new ParallelRaceGroup(
            new AimShooterAtAngle(angle, arm),
            new SolidBlue(leds));
    }

    /** Moves the arm back and spins up the flywheels to prepare for an amp shot. */
    Command prepAmpShot() {
        return new ParallelCommandGroup(
            new SpinFlywheels(8, 8, shooter),
            aimShooterAtAngle(100));
    }


    /** Resets the angle and speed of the shooter back to its default idle position. */
    Command resetShooter() {
        return new ParallelCommandGroup(
            new SpinFlywheels(0, 0, shooter),
            aimShooterAtAngle(ArmConstants.armMinAngleDegrees+5));
    }

    /** Spins the flywheels up to speed and aims arm when pressed against the subwoofer. */
    Command prepSubwooferShot() {
        return new ParallelCommandGroup(
            new SpinFlywheels(12, 10, shooter),
            aimShooterAtAngle(60));
    }

    
    /** Command to prepare for a shot. Finishes after the flywheels spin up to a desired
     *  speed, and the shooter reaches the correct angle. */
    Command prepShotFromAnywhere() { 
        return new ParallelRaceGroup(
            new ParallelCommandGroup(
                new SpinFlywheels(24, 12, shooter),
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

    private void realBindings() {
        controller.rightTrigger()
            .whileTrue(intakeNote())
            .onFalse(indexNote());
    
        controller.leftTrigger().whileTrue(new ReverseIntake(intake, indexer));
        
        controller.rightBumper().onTrue(shootFromSubwoofer());
        controller.leftBumper()
            .onTrue(prepAmpShot())
            .onFalse(fireNote());

        
        
        controller.y().onTrue(new InstantCommand(() -> drivetrain.setRobotFacingForward()));


    }

    private void testBindings() {
        controller.rightTrigger()
            .whileTrue(intakeNote())
            .onFalse(indexNote());
    
            
        controller.leftTrigger().whileTrue(new ReverseIntake(intake, indexer));

        controller.rightBumper().whileTrue(new RaiseClimbArms(climb));
        controller.leftBumper().whileTrue(new LowerClimbArms(climb));
        
        controller.y().onTrue(new InstantCommand(() -> drivetrain.setRobotFacingForward()));

        // controller.a().onTrue(new AimShooterAtAngle(0, arm));
        // controller.b().onTrue(new AimShooterAtAngle(90, arm));
        // controller.x().onTrue(new AimShooterAtAngle(ArmConstants.armMaxAngleDegrees, arm));


        //controller.y().whileTrue(new SpinFlywheels(8, 8, shooter));
        controller.x().onTrue(resetShooter());
        controller.b().onTrue(shootFromAnywhere());
        controller.a().onTrue(shootFromSubwoofer());
        

        /** SYSID BINDINGS **/
        // controller.a().whileTrue(arm.generateSysIdQuasistatic(SysIdRoutine.Direction.kForward));
        // controller.b().whileTrue(arm.generateSysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        
        // controller.x().whileTrue(arm.generateSysIdDynamic(SysIdRoutine.Direction.kForward));
        // controller.y().whileTrue(arm.generateSysIdDynamic(SysIdRoutine.Direction.kReverse));

        
        //isRingInIntake.onTrue(new IndexNote(intake, indexer));    
    
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {

        // NamedCommands.registerCommand("shootFromSubwoofer", shootFromSubwoofer());
        // NamedCommands.registerCommand("shootFromAnywhere", shootFromAnywhere());
        // NamedCommands.registerCommand("indexNote", new IndexNote(intake, indexer));

        return AutoBuilder.buildAuto("3 Piece Amp Side");
    }
}
