// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climb.Climb;

public class LowerClimbArms extends Command {

    Climb climb;

    public LowerClimbArms(Climb climb) {
        this.climb=climb;
        addRequirements(climb);
    }

    @Override
    public void initialize() {
        climb.setVolts(-10);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {}

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        climb.setVolts(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
        //return climb.climbArmsAreDown();
    }
}
