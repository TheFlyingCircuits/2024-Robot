package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.VendorWrappers.Kraken;

public class Indexer extends SubsystemBase {

    private Kraken indexerMotor;
    private DigitalInput indexerProximitySwitchLeft;
    private DigitalInput indexerProximitySwitchRight;

    private PIDController indexerPID;
    private SimpleMotorFeedforward indexerFeedforward;

    public Indexer() {
        indexerMotor = new Kraken("indexer", ShooterConstants.indexerMotorID, "CTRENetwork");
        indexerProximitySwitchLeft = new DigitalInput(ShooterConstants.indexerProximitySwitchIDLeft);
        indexerProximitySwitchRight = new DigitalInput(ShooterConstants.indexerProximitySwitchIDRight);

        indexerPID = new PIDController(
            ShooterConstants.kPIndexerVoltsPerRPS,
            0, 
            0);
        
        indexerFeedforward = new SimpleMotorFeedforward(
            ShooterConstants.kSIndexerVolts,
            ShooterConstants.kVIndexerVoltsPerRPS);

        configMotor();
    }

    public void setBlackRollerSurfaceSpeed(double metersPerSecond) {
        // theoretical max of 5.65 meters per second of surface speed based on motor and gearing
        double blackRollerRotationsPerSecond = metersPerSecond * (1.0 / ShooterConstants.blackRollerCircumferenceMeters);
        double desiredMotorRotationsPerSecond = blackRollerRotationsPerSecond * (1.0 / ShooterConstants.indexerBlackRollerGearRatio);
        setMotorRotationsPerSecond(desiredMotorRotationsPerSecond);
    }

    public void setOrangeWheelsSurfaceSpeed(double metersPerSecond) {
        // theoretical max of 7.97 meters per second of surface speed based on motor and gearing
        double orangeWheelsRotationsPerSecond = metersPerSecond * (1.0 / ShooterConstants.orangeWheelsCircumferenceMeters);
        double desiredMotorRotationsPerSecond = orangeWheelsRotationsPerSecond * (1.0 / ShooterConstants.indexerOrangeWheelsGearRatio);
        setMotorRotationsPerSecond(desiredMotorRotationsPerSecond);
    }

    public void setMotorRotationsPerSecond(double desiredRotationsPerSecond) {
        double feedforwardOutput = indexerFeedforward.calculate(desiredRotationsPerSecond);

        double currentMotorSpeed = indexerMotor.getVelocity().getValueAsDouble();
        double pidOutput = indexerPID.calculate(currentMotorSpeed, desiredRotationsPerSecond);
        indexerMotor.setVoltage(feedforwardOutput + pidOutput);
    }

    /** max of 5.65 */
    public Command setBlackRollerSurfaceSpeedCommand(double metersPerSecond) {
        return this.run(() -> {this.setBlackRollerSurfaceSpeed(metersPerSecond);});
    }

    /** max of 7.97 */
    public Command setOrangeWheelsSurfaceSpeedCommand(double metersPerSecond) {
        return this.run(() -> {this.setOrangeWheelsSurfaceSpeed(metersPerSecond);});
    }

    public void setVolts(double volts) {
        if (volts == 0 && indexerPID.getSetpoint() != 0) {
            indexerPID.setSetpoint(0);
            // TODO: cleanup
        }
        indexerMotor.setVoltage(volts);
    }

    /**
     * Returns true if either of the proximity switches in the indexer detects a note present.
     */
    public boolean isNoteIndexed() {
        boolean leftSensorSeesNote = !indexerProximitySwitchLeft.get();
        boolean rightSensorSeesNote = !indexerProximitySwitchRight.get();
        return leftSensorSeesNote || rightSensorSeesNote;
    }

    private void configMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.CurrentLimits.StatorCurrentLimit = 250;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        indexerMotor.applyConfig(config);
    }

    @Override
    public void periodic() {
        Logger.recordOutput("indexer/amps", indexerMotor.getTorqueCurrent().getValueAsDouble());
        Logger.recordOutput("indexer/isNoteIndexed()", isNoteIndexed());
        Logger.recordOutput("indexer/indexerRPS", indexerMotor.getVelocity().getValueAsDouble());
        Logger.recordOutput("indexer/setpointRPS", indexerPID.getSetpoint()); 
        Logger.recordOutput("indexer/supplyCurrent", indexerMotor.getSupplyCurrent().getValueAsDouble());
    };

}
