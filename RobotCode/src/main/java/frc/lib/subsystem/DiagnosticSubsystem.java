package frc.lib.subsystem;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.subsystem.autodiagnose.AutoDiagnoseBase;
import frc.lib.subsystem.autodiagnose.AutoDiagnoseCANSparkMAX;
import frc.lib.subsystem.autodiagnose.AutoDiagnoseCANcoder;
import frc.lib.subsystem.autodiagnose.AutoDiagnoseCANivore;
import frc.lib.subsystem.autodiagnose.AutoDiagnoseKraken;
import frc.lib.subsystem.autodiagnose.AutoDiagnosePigeon2;
import frc.lib.subsystem.autodiagnose.AutoDiagnosePowerDistributionHub;
import frc.robot.VendorWrappers.Kraken;
import frc.robot.subsystems.AutoDiagnose;

public abstract class DiagnosticSubsystem extends SubsystemBase {
    public enum SubsystemStatus {
        OK,
        WARNING,
        ERROR,
    }

    private final String statusDirectory;
    private final boolean isReal;
    private final List<Fault> faults = new ArrayList<Fault>();
    private final List<AutoDiagnoseBase> devices = new ArrayList<AutoDiagnoseBase>();

    public DiagnosticSubsystem() {
        this.statusDirectory = "SubsystemStatus/" + this.getName();
        this.isReal = RobotBase.isReal();
        Command autoDiagnoseCommand = getAutoDiagnoseCommand();
        autoDiagnoseCommand.setName(this.getName() + "Check");
        SmartDashboard.putData(statusDirectory + "/SubsystemCheck", autoDiagnoseCommand);
        SmartDashboard.putBoolean(statusDirectory + "/RanCheck", false);

        addPeriodicRunnables();
    }

    public Command getAutoDiagnoseCommand() {
        return Commands.sequence(
                Commands.runOnce(() -> {
                    SmartDashboard.putBoolean(statusDirectory + "RanCheck", false);
                    clearAllFaults();
                    publishDiagnostics();
                }),
                autoDiagnoseCommand(),
                Commands.runOnce(() -> {
                    publishDiagnostics();
                    SmartDashboard.putBoolean(statusDirectory + "RanCheck", true);
                }));
    }

    private void addPeriodicRunnables() {
        AutoDiagnose.addPeriodicRunnable(this::checkForFaults, 0.25);
        AutoDiagnose.addPeriodicRunnable(this::publishDiagnostics, 1.0);

    }

    private void publishDiagnostics() {
        SubsystemStatus status = getSubsystemStatus();
        SmartDashboard.putString(statusDirectory + "/Status", status.name());

        String[] faultStrings = new String[this.faults.size()];
        for (int i = 0; i < this.faults.size(); i++) {
            Fault fault = this.faults.get(i);
            faultStrings[i] = String.format(("[%.2f] %s"), fault.timestamp, fault.message);
        }
        SmartDashboard.putStringArray(statusDirectory + "/Faults", faultStrings);
    }

    protected abstract Command autoDiagnoseCommand();

    public void addDevice(String label, CANcoder canCoder) {
        devices.add(new AutoDiagnoseCANcoder(label, canCoder));
    }

    public void addDevice(String label, String CANivoreName) {
        devices.add(new AutoDiagnoseCANivore(label, CANivoreName));
    }

    public void addDevice(String label, CANSparkMax sparkMax) {
        devices.add(new AutoDiagnoseCANSparkMAX(label, sparkMax));
    }

    public void addDevice(String label, Kraken kraken) {
        devices.add(new AutoDiagnoseKraken(label, kraken));
    }

    public void addDevice(String label, Pigeon2 pigeon) {
        devices.add(new AutoDiagnosePigeon2(label, pigeon));
    }

    public void addDevice(String label, PowerDistribution pdh) {
        devices.add(new AutoDiagnosePowerDistributionHub(label, pdh));
    }

    protected void addFault(Fault fault) {
        if (!this.faults.contains(fault)) {
            this.faults.add(fault);
        }
    }

    protected void addFault(String message, boolean isJustWarning) {
        this.addFault(new Fault(message, isJustWarning));
    }

    protected void addFaults(List<Fault> faults) {
        this.faults.addAll(faults);
    }

    public List<Fault> getFaults() {
        return this.faults;
    }

    public void clearAllFaults() {
        this.faults.clear();
    }

    public SubsystemStatus getSubsystemStatus() {
        SubsystemStatus worstStatus = SubsystemStatus.OK;

        for (Fault fault : this.faults) {
            if (fault.timestamp > Timer.getFPGATimestamp() - 10) {
                if (fault.isJustWarning) {
                    if (worstStatus != SubsystemStatus.ERROR) {
                        worstStatus = SubsystemStatus.WARNING;
                    } else {
                        worstStatus = SubsystemStatus.ERROR;
                    }
                }
            }
        }
        return worstStatus;
    }

    private void checkForFaults() {
        if (isReal) {
            for (AutoDiagnoseBase device : devices) {
                for (Fault fault : device.getFaultsList()) {
                    addFault(fault);
                }
            }
        }
    }
}
