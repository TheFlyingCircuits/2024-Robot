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

public class AimAtSpeakerWhileJoystickDrive extends Command {

    private Drivetrain drivetrain;
    private PIDController angleController;


    public AimAtSpeakerWhileJoystickDrive(Drivetrain drivetrain) {
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

    /**
   * Returns 0 if the parameter {@code num} is lower than {@code deadzone} to
   * prevent joystick drift.
   * 
   * @param num      Axis input value
   * @param deadzone Lowest value before input is set to 0
   * @return Axis input checked against deadzone value
   */
    private double deadzone(double num, double deadzone) {
        return Math.abs(num) > deadzone ? num : 0;
    }

    /**
   * Adds a deadzone to axis input and squares the input.
   * 
   * This function should always return a value between -1 and 1.
   * 
   * @param value Axis input
   * @return Squared and deadzoned input
   */
    private double modifyAxis(double value) {
        value = deadzone(value, ControllerConstants.controllerDeadzone);

        //clipping controller values so they aren't greater than 1
        value = value > 1 ? 1 : value;

        //adjust this power value for diffferences in how the robot handles (recommended between 1.5 and 3)
        return Math.signum(value) * Math.pow(Math.abs(value), 2.3);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        //Instead of modifying the x and y axis from the controller individually,
        //it was found it is better to combine them, modify, and reextract the X
        //and Y using trig wahoo
        double controllerX = -RobotContainer.controller.getLeftX();
        double controllerY = -RobotContainer.controller.getLeftY();
        double controllerXY = modifyAxis(Math.sqrt((controllerX*controllerX) + (controllerY*controllerY)));
        double theta = Math.atan2(controllerY, controllerX);
        double finalControllerX = controllerXY * Math.cos(theta);
        double finalControllerY = controllerXY * Math.sin(theta);

        if (DriverStation.getAlliance().get() == Alliance.Red) {
            finalControllerX*=-1;
            finalControllerY*=-1;
        }

        // Raw controller values after modifyAxis will be between -1 and 1.
        // Coefficient = maximum speed in meters or radians per second.
        ChassisSpeeds outputChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            finalControllerY*DrivetrainConstants.maxDesiredTeleopVelocityMetersPerSecond,
            finalControllerX*DrivetrainConstants.maxDesiredTeleopVelocityMetersPerSecond,
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
        return false;
    }
}
