package frc.lib.subsystem.autodiagnose;

import java.util.ArrayList;
import java.util.List;

import com.revrobotics.CANSparkMax;

import com.revrobotics.CANSparkBase.FaultID;

import frc.lib.VendorWrappers.Neo;
import frc.lib.subsystem.Fault;

public class AutoDiagnoseCANSparkMAX implements AutoDiagnoseBase {
    private CANSparkMax sparkMax;
        private final String label;

    public AutoDiagnoseCANSparkMAX(Neo sparkMax) {
        this.sparkMax = sparkMax;
        this.label = sparkMax.getName();
    }

    @Override
    public List<Fault> getFaultsList() {
        List<Fault> faults = new ArrayList<Fault>();

        // TODO: is this necessary ?
        if (sparkMax.getFault(FaultID.kBrownout)) {
            faults.add(new Fault(String.format("[%s]: Motor browned out", label), true));
        }
        if (sparkMax.getFault(FaultID.kCANRX)) {
            faults.add(new Fault(String.format("[%s]: CAN Recieve transmission fault", label), true));
        }
        if (sparkMax.getFault(FaultID.kCANTX)) {
            faults.add(new Fault(String.format("[%s]: CAN Recieve recieve fault", label), true));
        }
        if (sparkMax.getFault(FaultID.kDRVFault)) {
            faults.add(new Fault(String.format("[%s]: Gate driver fault", label), true));
        }
        if (sparkMax.getFault(FaultID.kEEPROMCRC)) {
            // TODO: email rev to find out what this is lol
            faults.add(new Fault(String.format("[%s]: EEPROMCRC", label), true));
        }
        if (sparkMax.getFault(FaultID.kMotorFault)) {
            faults.add(new Fault(String.format("[%s]: Motor fault", label), true));
        }
        if (sparkMax.getFault(FaultID.kSensorFault)) {
            faults.add(new Fault(String.format("[%s]: Sensor fault", label), true));
        }

        return faults;
    }

}
