package frc.lib.subsystem.autodiagnose;

import java.util.ArrayList;
import java.util.List;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import frc.lib.subsystem.Fault;

public class AutoDiagnoseCANcoder implements AutoDiagnoseBase {
    private final String label;
    private final StatusSignal<Boolean> badMagnetSignal;
    private final StatusSignal<Boolean> bootEnabledSignal;
    private final StatusSignal<Boolean> hardwareFaultSignal;
    private final StatusSignal<Boolean> undervoltageSignal;

    public AutoDiagnoseCANcoder(String label, CANcoder canCoder) {
        this.label = label;

        this.badMagnetSignal = canCoder.getFault_BadMagnet();
        this.bootEnabledSignal = canCoder.getFault_BootDuringEnable();
        this.hardwareFaultSignal = canCoder.getFault_Hardware();
        this.undervoltageSignal = canCoder.getFault_Undervoltage();
    }
    
    @Override
    public List<Fault> getFaultsList() {
        List<Fault> faults = new ArrayList<Fault>();

        if(badMagnetSignal.refresh().getValue()) {
            faults.add(new Fault(String.format("[%s]: Bad magnet distance/magnet missing", label), false));
        }
        if(bootEnabledSignal.refresh().getValue()) {
            faults.add(new Fault(String.format("[%s]: Device booted while enabled", label), false));
        }
        if(hardwareFaultSignal.refresh().getValue()) {
            faults.add(new Fault(String.format("[%s]: Hadware fault", label), false));
        }
        if(undervoltageSignal.refresh().getValue()) {
            faults.add(new Fault(String.format("[%s]: Bad magnet distance/magnet missing", label), false));
        }

        return faults;
    }
    
}
