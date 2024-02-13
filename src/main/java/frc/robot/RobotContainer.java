// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.arm.AimShooterAtSpeaker;
import frc.robot.commands.drivetrain.AimAtSpeakerWhileJoystickDrive;
import frc.robot.commands.drivetrain.JoystickDrive;
import frc.robot.commands.intake.IndexNote;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIO;
import frc.robot.subsystems.arm.ArmIOSim;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.GyroIO;
import frc.robot.subsystems.drivetrain.GyroIOPigeon;
import frc.robot.subsystems.drivetrain.GyroIOSim;
import frc.robot.subsystems.drivetrain.SwerveModuleIO;
import frc.robot.subsystems.drivetrain.SwerveModuleIONeo;
import frc.robot.subsystems.drivetrain.SwerveModuleIOSim;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Indexer;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonLib;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
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

            shooter = new Shooter(new ShooterIO() {});

            arm = new Arm(new ArmIO() {});
        }
        else {

            drivetrain = new Drivetrain(
                new GyroIOSim(),
                new SwerveModuleIOSim() {},
                new SwerveModuleIOSim() {},
                new SwerveModuleIOSim() {},
                new SwerveModuleIOSim() {},
                new VisionIO() {}
            );

            shooter = new Shooter(new ShooterIO() {});

            arm = new Arm(new ArmIOSim());
        }
        

        intake = new Intake();
        indexer = new Indexer();

        configureBindings();
        
        drivetrain.setDefaultCommand(new JoystickDrive(true, drivetrain));
        //arm.setDefaultCommand(new InstantCommand(() -> arm.setArmDesiredPosition(0), arm));
        isRingInIntake = new Trigger(intake::isRingInIntake);
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
        controller.y().onTrue(new InstantCommand(() -> drivetrain.setRobotRotation2d(new Rotation2d(0))));

        controller.b().onTrue(new InstantCommand(() -> arm.setArmDesiredPosition(30)));

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
        PathPlannerPath path = PathPlannerPath.fromPathFile("path2");
        return AutoBuilder.buildAuto("Vision Test Path");
    }
}
