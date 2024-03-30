package frc.lib.subsystem.autodiagnose;

import frc.lib.subsystem.Fault;
import java.util.List;

public interface AutoDiagnoseBase {
    List<Fault> getFaultsList();
}
