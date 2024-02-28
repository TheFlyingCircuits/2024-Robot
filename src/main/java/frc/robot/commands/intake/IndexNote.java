// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Indexer;

/** Add your docs here. */
public class IndexNote extends Command {


    private Intake intake;
    private Indexer indexer;

    public IndexNote(Intake intake, Indexer indexer) {
        this.intake = intake;
        this.indexer = indexer;
        
        addRequirements(intake, indexer);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        intake.setVolts(5);
        indexer.setIndexerRPM(800);
    }

    @Override
    public void end(boolean interrupted) {
        intake.setVolts(0);
        indexer.setIndexerRPM(0);
    }

    @Override
    public boolean isFinished() {
        return indexer.isNoteIndexed();
    }
}
