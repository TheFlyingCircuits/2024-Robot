package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Indexer;

public class FireNote extends Command {
    
    private Indexer indexer;
    private Timer timer;

    /** Pulses the indexer wheels to bring a note from the indexer into the shooter. */
    public FireNote(Indexer indexer) {
        this.indexer = indexer;

        timer = new Timer();
    }

    @Override
    public void initialize() {
        indexer.setVolts(1);
        timer.restart();
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean isInterrupted) {
        indexer.setVolts(0);
    }
    
    @Override
    public boolean isFinished() {
        return timer.get() > 0.5;
    }
}
