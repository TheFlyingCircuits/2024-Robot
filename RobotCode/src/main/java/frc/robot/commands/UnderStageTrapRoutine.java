// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
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
public class UnderStageTrapRoutine extends SequentialCommandGroup {

    public UnderStageTrapRoutine(Supplier<ChassisSpeeds> translationController, Climb climb, Arm arm, Shooter shooter, Indexer indexer, LEDs leds, Drivetrain drivetrain) {
        addCommands(
            
            //while we are driving, do this sequence
            drivetrain.run(() -> {
                Pose2d nearestTrap = Constants.FieldElement.getClosestTrap(drivetrain.getPoseMeters());
                drivetrain.fieldOrientedDriveOnALine(translationController.get(), nearestTrap);
            })
            .raceWith(
                new WaitUntilCommand(() -> {
                    Pose2d nearestTrap = Constants.FieldElement.getClosestTrap(drivetrain.getPoseMeters());
                    Translation2d trapAxis = new Translation2d(nearestTrap.getRotation().getCos(), nearestTrap.getRotation().getSin());
                    Translation2d trapLocation = nearestTrap.getTranslation();
                    Translation2d robotLocation = drivetrain.getPoseMeters().getTranslation();
                    Translation2d robotToTrap = robotLocation.minus(trapLocation);
                    double signedDistanceAlongTrapAxis = robotToTrap.getX() * trapAxis.getX() + robotToTrap.getY() * trapAxis.getY();
                    return (signedDistanceAlongTrapAxis < 0) && (robotToTrap.getNorm() < Units.inchesToMeters(3));
                })
                .andThen(arm.run(() -> {
                    Pose2d nearestTrap = Constants.FieldElement.getClosestTrap(drivetrain.getPoseMeters());
                    Translation2d trapAxis = new Translation2d(nearestTrap.getRotation().getCos(), nearestTrap.getRotation().getSin());
                    Translation2d trapLocation = nearestTrap.getTranslation();
                    Translation2d robotLocation = drivetrain.getPoseMeters().getTranslation();
                    Translation2d robotToTrap = robotLocation.minus(trapLocation);
                    double signedDistanceAlongTrapAxis = robotToTrap.getX() * trapAxis.getX() + robotToTrap.getY() * trapAxis.getY();
                    double pivotDistanceAlongTrapAxis = signedDistanceAlongTrapAxis - ArmConstants.pivotOffsetMeters;
                    double desiredFlywheeDistance = Units.inchesToMeters(10);
                    double cosine = (desiredFlywheeDistance - pivotDistanceAlongTrapAxis) / ArmConstants.armLengthMeters;
                    cosine = MathUtil.clamp(cosine, -1, 1);

                    double desiredRadians = Math.acos(cosine);
                    double desiredDegrees = Math.toDegrees(desiredRadians);


                    // if (pivotDistanceAlongTrapAxis < 0) {
                    //     arm.setDesiredDegrees(0);
                    //     return;
                    // }

                    arm.setDesiredDegrees(desiredDegrees);

                }))
                .alongWith(
                    shooter.setFlywheelSurfaceSpeedCommand(1)
                )
                .raceWith(new WaitUntilCommand(() -> {
                    return arm.getDegrees() >= 95;
                }))
                .andThen(new ParallelCommandGroup(
                            new InstantCommand(() -> {arm.setDisableSetpointChecking(true);}),
                            arm.holdCurrentPositionCommand().until(() -> {return arm.getDegrees() <= 80;}),
                            climb.raiseHooksCommand().until(climb::climbArmsUp)
                        )
                ).andThen(
                    new InstantCommand(() -> {arm.setDisableSetpointChecking(false);})
                ).andThen(new SequentialCommandGroup(
                            //once we are on the chain, start climbing
                            new ParallelRaceGroup(
                                climb.lowerHooksCommand().until(climb::climbArmsDown),
                                arm.setDesiredDegreesCommand(95),
                                shooter.setFlywheelSurfaceSpeedCommand(10)
                            ),

                            new WaitCommand(0.5), // TODO: are we sinking here because the climb command finishes when arms are down?

                            //fire the note at the top
                            indexer.setOrangeWheelsSurfaceSpeedCommand(7).withTimeout(1)
                                .raceWith(shooter.setFlywheelSurfaceSpeedCommand(10))
                                .alongWith(new ScheduleCommand(leds.playFireNoteAnimationCommand())),

                            climb.raiseHooksCommand(4).withTimeout(0.5)
                          )
                )
            )
        );
                
    }


}