// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.FieldElement;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.shooter.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class UnderStageTrapRoutine extends SequentialCommandGroup {

    private Drivetrain drivetrain;
    private Supplier<ChassisSpeeds> translationController;

    public UnderStageTrapRoutine(Supplier<ChassisSpeeds> translationController, Climb climb, Arm arm, Shooter shooter, Drivetrain drivetrain, Supplier<Command> fireNoteCommand) {
        this.drivetrain = drivetrain;
        this.translationController = translationController;
        
        addCommands(
            new ParallelRaceGroup(
                snapToTrap(),
                new SequentialCommandGroup(
                    // Wait until we're in position to start raising the arm
                    new WaitUntilCommand(() -> {return getTrapToRobot().getNorm() < Units.inchesToMeters(3);}),
                    // Raise the arm as we drive forward
                    new ParallelRaceGroup(
                        // Start raising the arm as we move out from under the stage
                        arm.run(() -> {
                            double chainDistanceFromTrap = Units.inchesToMeters(12 + 4 + (5./8.));
                            double chainHeight = Units.inchesToMeters((2 * 12) + 4 + (1./4.));
                            double deltaY = chainHeight - ArmConstants.pivotHeightMeters;
                            double deltaX = chainDistanceFromTrap - getPivotDistanceAlongTrapAxis();
                            arm.setDesiredDegrees(Math.toDegrees(Math.atan2(deltaY, deltaX)));
                        }).until(() -> {return arm.getDegrees() >= 95;}),
                        // Slowly spin the flywheels to help the chain pass over the arm.
                        shooter.setFlywheelSurfaceSpeedCommand(1)
                    ),
                    // Drive back into the stage to trigger the climb
                    new ParallelCommandGroup(
                        // Raise the hooks
                        climb.raiseHooksCommand().until(climb::climbArmsUp),
                        
                        // Push the arm into the stage until we reach the trigger angle.
                        new SequentialCommandGroup(
                            arm.setCoastCommand(true),
                            arm.holdCurrentPositionCommand().until(() -> {return arm.getDegrees() <= 80;}),
                            arm.setCoastCommand(false)
                        )
                    ),
                    // Start the climb
                    new ParallelRaceGroup(
                        climb.lowerHooksCommand().until(climb::climbArmsDown),
                        arm.setDesiredDegreesCommand(95),
                        shooter.setFlywheelSurfaceSpeedCommand(10)
                    ),
                    // Wait for the swinging to stop
                    new WaitCommand(0.5), // TODO: are we sinking here because the climb command finishes when arms are down?
                    // Score in the trap
                    fireNoteCommand.get(),
                    // Lower down a tad so we're not hanging on the trap opening
                    climb.raiseHooksCommand(4).withTimeout(0.5)
                )
            )
        );    
    }

    private Command snapToTrap() {
        return drivetrain.run(() -> {
            Pose2d nearestTrap = FieldElement.getClosestTrap(drivetrain.getPoseMeters());
            drivetrain.fieldOrientedDriveOnALine(translationController.get(), nearestTrap);
        });
    }

    private Translation2d getTrapToRobot() {
        Pose2d robot = drivetrain.getPoseMeters();
        Pose2d nearestTrap = FieldElement.getClosestTrap(robot);
        return robot.getTranslation().minus(nearestTrap.getTranslation());
    }

    private double getPivotDistanceAlongTrapAxis() {
        Pose2d nearestTrap = FieldElement.getClosestTrap(drivetrain.getPoseMeters());
        Translation2d trapAxis = new Translation2d(nearestTrap.getRotation().getCos(), nearestTrap.getRotation().getSin());
        Translation2d robotToTrap = this.getTrapToRobot();
        double robotDistanceAlongTrapAxis = robotToTrap.getX() * trapAxis.getX() + robotToTrap.getY() * trapAxis.getY();
        double pivotDistanceAlongTrapAxis = robotDistanceAlongTrapAxis - ArmConstants.pivotOffsetMeters;
        return pivotDistanceAlongTrapAxis;
    }


}
