package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Indexer extends SubsystemBase {

    private TalonFX indexerMotor;
    private DigitalInput indexerProximitySwitch;

    public Indexer() {
        indexerMotor = new TalonFX(ShooterConstants.indexerMotorID, "CTRENetwork");
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
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        indexerMotor.getConfigurator().apply(config);
    }

    @Override
    public void periodic() {};

}
