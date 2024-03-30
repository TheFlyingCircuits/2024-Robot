package frc.lib.subsystem;

import edu.wpi.first.wpilibj.Timer;

public class RunnablePeriodic {
    public final Runnable toRun;
    public final double periodSeconds;
    public final Timer timer;

    public RunnablePeriodic(Runnable toRun, double periodSeconds) {
        this.toRun = toRun;
        this.periodSeconds = periodSeconds;
        timer = new Timer();
    }
}
