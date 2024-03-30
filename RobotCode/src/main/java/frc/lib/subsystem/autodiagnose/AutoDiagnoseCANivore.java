package frc.lib.subsystem.autodiagnose;

import com.ctre.phoenix6.StatusCode;
import frc.lib.subsystem.Fault;
import com.ctre.phoenix6.CANBus.CANBusStatus;

import java.util.ArrayList;
import java.util.List;
import com.ctre.phoenix6.CANBus;


public class AutoDiagnoseCANivore implements AutoDiagnoseBase {
    // TODO: verify real name of our canivore
    private final String label;
    private CANBusStatus canivoreInfo;

    public AutoDiagnoseCANivore(String label, String CANivoreName) {
        this.label = label;
        this.canivoreInfo = CANBus.getStatus(CANivoreName);
    }

    @Override
    public List<Fault> getFaultsList() {
        List<Fault> faults = new ArrayList<>();

        StatusCode canivorestatus = canivoreInfo.Status;
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getFaultsList'");
    }
}
