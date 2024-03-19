// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.leds.LEDs;
import frc.robot.subsystems.shooter.Indexer;
import frc.robot.subsystems.shooter.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TrapRoutine extends SequentialCommandGroup {

    public TrapRoutine(Supplier<ChassisSpeeds> translationController, Climb climb, Arm arm, Shooter shooter, Indexer indexer, LEDs leds, Drivetrain drivetrain) {
        addCommands(
            
            //while we are driving, do this sequence
            new AimEverythingAtTrap(drivetrain, translationController)
                .raceWith(new SequentialCommandGroup(
                    //raise the hooks and lean the arm back
                    new ParallelCommandGroup(
                        climb.raiseHooksCommand(),
                        new ParallelRaceGroup(
                            arm.setDesiredDegreesCommand(ArmConstants.armMaxAngleDegrees),
                            new WaitCommand(0.2).andThen(new WaitUntilCommand(arm::isCloseToTarget))
                        )
                    ),
                    //go into coast mode, spin the flywheels to move the chain over them
                    new InstantCommand(() -> {arm.setDisableSetpointChecking(true);}),
                    new ParallelRaceGroup(
                        arm.holdCurrentPositionCommand().until(() -> {return arm.getDegrees() <= 76;}),
                        shooter.setFlywheelSurfaceSpeedCommand(1, 1)
                    ),
                    new InstantCommand(() -> {arm.setDisableSetpointChecking(false);})
                )
            ),

            //once we are on the chain, start climbing
            new ParallelRaceGroup(
                climb.lowerHooksCommand().until(climb::climbArmsDown),
                arm.setDesiredDegreesCommand(90),
                shooter.setFlywheelSurfaceSpeedCommand(10)
            ),

            //fire the note at the top
            indexer.setOrangeWheelsSurfaceSpeedCommand(7).withTimeout(0.4)
               .alongWith(new ScheduleCommand(leds.playFireNoteAnimationCommand())),

            new WaitCommand(1),

            climb.raiseHooksCommand(2).withTimeout(0.5)
        );
    }
}
