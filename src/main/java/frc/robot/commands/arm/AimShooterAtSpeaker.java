// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class AimShooterAtSpeaker extends Command {

    Arm arm;
    Drivetrain drivetrain;
    
    public AimShooterAtSpeaker(Arm arm, Drivetrain drivetrain) {
        this.arm = arm;
        this.drivetrain = drivetrain;

        addRequirements(arm);

    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        double targetAngleRadians = Math.atan2(
            FieldConstants.speakerHeightMeters - FieldConstants.pivotHeightMeters,
                                              // 0.22 is meter distance from center of robot to the pivot point
            drivetrain.distToSpeakerBaseMeters()+0.22);

        arm.setArmDesiredPosition(Math.toDegrees(targetAngleRadians));
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return !arm.isMovingToTarget;
    }
}
