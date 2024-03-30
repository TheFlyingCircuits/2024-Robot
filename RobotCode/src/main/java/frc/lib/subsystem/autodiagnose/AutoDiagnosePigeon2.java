package frc.lib.subsystem.autodiagnose;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import frc.lib.subsystem.Fault;

public class AutoDiagnosePigeon2 implements AutoDiagnoseBase {
    private final String label;
    private final StatusSignal<Integer> firmwareVersionSignal;
    private final StatusSignal<Boolean> bootDuringEnableSignal;
    private final StatusSignal<Boolean> bootIntoMotionSignal;
    private final StatusSignal<Boolean> bootupAccelerometerSignal;
    private final StatusSignal<Boolean> bootupGyroscopeSignal;
    private final StatusSignal<Boolean> hardwareSignal;

    public AutoDiagnosePigeon2(String label, Pigeon2 pigeon) {
        this.label = label;

        this.firmwareVersionSignal = pigeon.getVersion();
        this.bootDuringEnableSignal = pigeon.getFault_BootDuringEnable();
        this.bootIntoMotionSignal = pigeon.getFault_BootIntoMotion();
        this.bootupAccelerometerSignal = pigeon.getFault_BootupAccelerometer();
        this.bootupGyroscopeSignal = pigeon.getFault_BootupGyroscope();
        this.hardwareSignal = pigeon.getFault_Hardware();
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
        if(bootIntoMotionSignal.refresh().getValue()) {
            faults.add(new Fault(String.format("[%s]: Device booted while in motion", label), true));
        }
        if(bootupAccelerometerSignal.refresh().getValue()) {
            faults.add(new Fault(String.format("[%s]: Accelerometer boot checks failed", label), true));
        }
        if(bootupGyroscopeSignal.refresh().getValue()) {
            faults.add(new Fault(String.format("[%s]: Gyroscope boot checks failed", label), true));
        }
        if(hardwareSignal.refresh().getValue()) {
            faults.add(new Fault(String.format("[%s]: Hadware fault", label), true));
        }

        return faults;
    }
}
