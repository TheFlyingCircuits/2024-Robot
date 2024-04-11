// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.subsystem.RunnablePeriodic;

public class AutoDiagnose extends SubsystemBase {
  
    private static List<RunnablePeriodic> runnablePeriodics = new ArrayList<RunnablePeriodic>();


    /** Creates a new AutoDiagnose. */
    public AutoDiagnose() {}

    public static void addPeriodicRunnable(Runnable toRun, double periodSeconds) {
        runnablePeriodics.add(new RunnablePeriodic(toRun, periodSeconds));
    }

  
    @Override
    public void periodic() {
        for (RunnablePeriodic runnablePeriodic : runnablePeriodics) {
            if(runnablePeriodic.timer.hasElapsed(runnablePeriodic.periodSeconds)) {
                runnablePeriodic.toRun.run();
            }
        }
    }
}
