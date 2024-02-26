package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.VendorWrappers.Kraken;

public class Indexer extends SubsystemBase {

    private Kraken indexerMotor;
    private DigitalInput indexerProximitySwitch;
    //Position never used, only read for error detection
    private double motorPosition;

    private PIDController indexerPID;
    private SimpleMotorFeedforward indexerFeedforward;

    public Indexer() {
        indexerMotor = new Kraken(ShooterConstants.indexerMotorID, "CTRENetwork");
        indexerProximitySwitch = new DigitalInput(ShooterConstants.indexerProximitySwitchID);

        indexerPID = new PIDController(
            ShooterConstants.kPIndexerVoltsPerRPM,
            0, 
            0);
        
        indexerFeedforward = new SimpleMotorFeedforward(
            ShooterConstants.kSIndexerVolts,
            ShooterConstants.kVIndexerVoltsPerRPM);

        configMotor();
    }

    public void setIndexerRPM(double rpm) {
        double feedforwardOutput = indexerFeedforward.calculate(rpm);
        double pidOutput = indexerPID.calculate(getIndexerRPM(), rpm);
        indexerMotor.setVoltage(feedforwardOutput + pidOutput);
    }

    /** Gets the RPM of the indexer wheels. */
    public double getIndexerRPM() {
        return indexerMotor.getVelocity().getValueAsDouble()/ShooterConstants.indexerGearReduction*60;
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
        config.CurrentLimits.StatorCurrentLimit = 120;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        indexerMotor.applyConfig(config);
    }

    @Override
    public void periodic() {
        Logger.recordOutput("indexer/isNoteIndexed()", isNoteIndexed());
        Logger.recordOutput("indexer/indexerRPM", getIndexerRPM());
        Logger.recordOutput("indexer/setpointRPM", indexerPID.getSetpoint());

        //Position never used, only read for error detection
        motorPosition = indexerMotor.getPosition().getValueAsDouble();
        
    };

}
