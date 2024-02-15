// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ArmConstants;
import frc.robot.commands.arm.AimShooterAtAngle;
import frc.robot.commands.arm.AimShooterAtSpeaker;
import frc.robot.commands.drivetrain.AimAtSpeakerWhileJoystickDrive;
import frc.robot.commands.drivetrain.JoystickDrive;
import frc.robot.commands.intake.IndexNote;
import frc.robot.commands.intake.IntakeNote;
import frc.robot.commands.shooter.FireNote;
import frc.robot.commands.shooter.SpinFlywheels;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIO;
import frc.robot.subsystems.arm.ArmIOSim;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.GyroIOPigeon;
import frc.robot.subsystems.drivetrain.GyroIOSim;
import frc.robot.subsystems.drivetrain.SwerveModuleIONeo;
import frc.robot.subsystems.drivetrain.SwerveModuleIOSim;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Indexer;
import frc.robot.subsystems.shooter.Shooter;
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

    private Trigger isRingInIntake;
    
    public RobotContainer() {

        if (RobotBase.isReal()) {
            drivetrain = new Drivetrain(
                new GyroIOPigeon(),
                new SwerveModuleIONeo(1, 2, -0.177978515625, 0),
                new SwerveModuleIONeo(3, 4, 0.33935546875, 1),
                new SwerveModuleIONeo(5, 6, -0.339599609375, 2),
                new SwerveModuleIONeo(7, 8, -0.206787109375, 3),
                new VisionIOPhotonLib()
            );

            shooter = new Shooter(new ShooterIOKraken());

            arm = new Arm(new ArmIO() {});

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
        
        
        drivetrain.setDefaultCommand(new JoystickDrive(true, drivetrain));

        isRingInIntake = new Trigger(intake::isRingInIntake);
        
        configureBindings();
    }


    //these command compositions must be separated into their own methods
    //since wpilib requires a new instance of a command to be used in each
    //composition (the same instance of a command cannot be used in multiple compositions).


    /** Moves the arm back and spins up the flywheels to prepare for an amp shot. */
    ParallelCommandGroup prepAmpShot() {
        return new ParallelCommandGroup(
            new SpinFlywheels(500, 500, shooter),
            new AimShooterAtAngle(100, arm));
    }


    /** Resets the angle and speed of the shooter back to its default idle position. */
    ParallelCommandGroup resetShooter() {
        return new ParallelCommandGroup(
            new SpinFlywheels(0, 0, shooter),
            new InstantCommand(() -> arm.setArmDesiredPosition(ArmConstants.kArmMinAngleDegrees)));
    }

    /** Spins the flywheels up to speed and aims arm when pressed against the subwoofer. */
    ParallelCommandGroup prepSubwooferShot() {
        return new ParallelCommandGroup(
            new SpinFlywheels(500, 500, shooter),
            new AimShooterAtAngle(70, arm));
    }

    
    /** Command to prepare for a shot. Finishes after the flywheels spin up to a desired
     *  speed, and the shooter reaches the correct angle. */
    ParallelRaceGroup prepShotFromAnywhere() { 
        return new ParallelRaceGroup(
            new ParallelCommandGroup(
                new SpinFlywheels(500, 500, shooter),
                new AimShooterAtSpeaker(arm, drivetrain)),
            new AimAtSpeakerWhileJoystickDrive(drivetrain));
    }

    //aims and then shoots in one motion
    SequentialCommandGroup shootFromSubwoofer() { 
        return new SequentialCommandGroup(
            prepSubwooferShot(),
            new FireNote(indexer),
            resetShooter());
    }


    //aims and then shoots in one motion
    SequentialCommandGroup shootFromAnywhere() {
        return new SequentialCommandGroup(
            prepShotFromAnywhere(),
            new FireNote(indexer),
            resetShooter());
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
     * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {
        controller.rightTrigger().whileTrue(new IntakeNote(intake));

        controller.b().onTrue(shootFromAnywhere());
        controller.rightBumper().onTrue(shootFromSubwoofer());


        controller.y().onTrue(new InstantCommand(() -> drivetrain.setRobotFacingForward()));
        controller.a().toggleOnTrue(new AimShooterAtSpeaker(arm, drivetrain));
        controller.x().toggleOnTrue(new AimAtSpeakerWhileJoystickDrive(drivetrain));


        isRingInIntake.onTrue(new IndexNote(intake, indexer));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {

        NamedCommands.registerCommand("shootFromSubwoofer", shootFromSubwoofer());
        NamedCommands.registerCommand("shootFromAnywhere", shootFromAnywhere());
        NamedCommands.registerCommand("indexNote", new IndexNote(intake, indexer));

        return AutoBuilder.buildAuto("Vision Test Path");
    }
}
