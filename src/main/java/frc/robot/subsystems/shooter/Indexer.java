package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Indexer extends SubsystemBase {

    private CANSparkMax indexerMotor;
    private DigitalInput indexerProximitySwitch;

    public Indexer() {
        indexerMotor = new CANSparkMax(ShooterConstants.indexerMotorID, MotorType.kBrushless);
        indexerProximitySwitch = new DigitalInput(ShooterConstants.indexerProximitySwitchID);

        configMotor();
    }


    public void setVolts(double volts) {
        indexerMotor.setVoltage(volts);
    }

    /**
     * Returns true if the proximity switch in the indexer detects a note present.
     */
    public boolean isNoteIndexed() {
        return !indexerProximitySwitch.get();
    }

    private void configMotor() {
        indexerMotor.setInverted(false);

        indexerMotor.setIdleMode(IdleMode.kBrake);

        indexerMotor.burnFlash();
    }

    @Override
    public void periodic() {};

}
