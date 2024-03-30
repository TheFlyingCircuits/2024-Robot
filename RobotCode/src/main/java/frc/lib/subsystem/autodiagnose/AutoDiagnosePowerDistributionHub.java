package frc.lib.subsystem.autodiagnose;

import java.util.List;
import edu.wpi.first.wpilibj.PowerDistribution;
import frc.lib.subsystem.Fault;

public class AutoDiagnosePowerDistributionHub implements AutoDiagnoseBase {
    private String label;
    private PowerDistribution pdh;
    // TODO: all of this, email rev for fault help decoding shorts
    public AutoDiagnosePowerDistributionHub(String label, PowerDistribution pdh) {
        this.label = label;
        this.pdh = pdh;

        pdh.getType();
        pdh.getFaults();
        
    }

    @Override
    public List<Fault> getFaultsList() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getFaultsList'");
    }
}