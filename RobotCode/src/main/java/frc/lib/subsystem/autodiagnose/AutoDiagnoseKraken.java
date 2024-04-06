package frc.lib.subsystem.autodiagnose;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;

import frc.lib.VendorWrappers.Kraken;
import frc.lib.subsystem.Fault;

public class AutoDiagnoseKraken implements AutoDiagnoseBase {

    private String label;

    private final StatusSignal<Integer> firmwareVersionSignal;
    private final StatusSignal<Boolean> bootDuringEnableSignal;
    private final StatusSignal<Boolean> deviceTempSignal;
    private final StatusSignal<Boolean> hardwareSignal;
    private final StatusSignal<Boolean> procTempSignal;



    public AutoDiagnoseKraken(Kraken kraken) {
        this.label = kraken.getName();

        this.firmwareVersionSignal = kraken.getVersion();
        this.bootDuringEnableSignal = kraken.getFault_BootDuringEnable();
        this.deviceTempSignal = kraken.getFault_DeviceTemp();
        this.hardwareSignal = kraken.getFault_Hardware();
        this.procTempSignal = kraken.getFault_ProcTemp();
    }

    @Override
    public List<Fault> getFaultsList() {
        List<Fault> faults = new ArrayList<Fault>();
        
        if(firmwareVersionSignal.refresh().getStatus() != StatusCode.OK) {
            faults.add(new Fault(String.format("[%s]: No communication with device", label), true));
        }
        if(bootDuringEnableSignal.refresh().getValue()) {
            faults.add(new Fault(String.format("[%s]: Device booted while enabled", label), true));
        }
        if(deviceTempSignal.refresh().getValue()) {
            faults.add(new Fault(String.format("[%s]: Device temperature exceeds limit", label), true));
        }
        if(hardwareSignal.refresh().getValue()) {
            faults.add(new Fault(String.format("[%s]: Hadware fault", label), true));
        }
        if(procTempSignal.refresh().getValue()) {
            faults.add(new Fault(String.format("[%s]: Processor temperature exceeds limit", label), true));
        }

        return faults;
    }
}
