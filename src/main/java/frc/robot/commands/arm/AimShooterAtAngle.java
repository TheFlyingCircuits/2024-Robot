// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;

public class AimShooterAtAngle extends Command {

    double targetAngleDegrees;
    Arm arm;
    
    public AimShooterAtAngle(double targetAngleDegrees, Arm arm) {
        this.arm = arm;
        this.targetAngleDegrees = targetAngleDegrees;

        addRequirements(arm);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        arm.setArmDesiredPosition(targetAngleDegrees);
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
