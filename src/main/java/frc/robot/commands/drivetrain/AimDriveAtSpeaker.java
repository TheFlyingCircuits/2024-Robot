// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class AimDriveAtSpeaker extends Command {

    private Drivetrain drivetrain;
    private PIDController angleController;


    public AimDriveAtSpeaker(Drivetrain drivetrain) {
        addRequirements(drivetrain);
        this.drivetrain = drivetrain;
        
        angleController = new PIDController(
            4, 0, 0);

        angleController.enableContinuousInput(-180, 180);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

    }


    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        ChassisSpeeds outputChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            0,
            0,
            Math.toRadians(angleController.calculate(
                drivetrain.getRobotRotation2d().getDegrees(),
                drivetrain.getDriveAngleToSpeaker().getDegrees()
                )),
            drivetrain.getRobotRotation2d()    
            );

        drivetrain.drive(outputChassisSpeeds, true);

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        drivetrain.drive(new ChassisSpeeds(0, 0, 0), true);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return Math.abs(drivetrain.getRobotRotation2d().minus(drivetrain.getDriveAngleToSpeaker()).getDegrees()) < 5;
    }
}
