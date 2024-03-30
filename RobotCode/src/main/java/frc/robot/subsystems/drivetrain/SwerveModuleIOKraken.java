package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.SwerveModuleConstants;
import frc.robot.VendorWrappers.Kraken;
import frc.robot.VendorWrappers.Neo;

public class SwerveModuleIOKraken implements SwerveModuleIO {
    private CANcoder absoluteEncoder;
    private Neo angleMotor;
    private Kraken driveMotor;
    /**
     * 
     * @param driveMotorID - ID of the drive motor
     * @param angleMotorID - ID of the angle motor
     * @param angleOffsetDegrees - Offset of the angle motor, in degrees
     * @param cancoderID - ID of the absolute CANcoder mounted ontop of the swerve
     * @param isDriveMotorOnTop - Is drive motor mounted on top
     * @param isAngleMotorOnTop - Is angle motor mounted on top
     */
    public SwerveModuleIOKraken(int driveMotorID, int angleMotorID, double angleOffsetDegrees, int cancoderID, boolean isDriveMotorOnTop, boolean isAngleMotorOnTop, String name){
        
        /* Angle Encoder Config */
        absoluteEncoder = new CANcoder(cancoderID, "CTRENetwork");
        configCANCoder(angleOffsetDegrees);

        /* Angle Motor Config */
        angleMotor = new Neo(name+"Steer", angleMotorID);
        if(isAngleMotorOnTop) {
            configAngleMotor(false);
        } else {
            configAngleMotor(true);
        }

        /* Drive Motor Config */
        driveMotor = new Kraken(name+"Drive", driveMotorID, "CTRENetwork");
        if(isDriveMotorOnTop) {
            configDriveMotor(InvertedValue.CounterClockwise_Positive);
        } else {
            configDriveMotor(InvertedValue.Clockwise_Positive);
        }
    }

    @Override
    public void updateInputs(SwerveModuleIOInputs inputs) {
        inputs.drivePositionMeters = driveMotor.getPosition().getValueAsDouble() * (SwerveModuleConstants.driveGearReduction * SwerveModuleConstants.wheelCircumferenceMeters);
        inputs.driveVelocityMetersPerSecond = driveMotor.getVelocity().getValueAsDouble() * (SwerveModuleConstants.driveGearReduction * SwerveModuleConstants.wheelCircumferenceMeters);
        inputs.angleAbsolutePositionDegrees = absoluteEncoder.getAbsolutePosition().getValueAsDouble()*360;

        inputs.driveAppliedVoltage = driveMotor.getMotorVoltage().getValueAsDouble();
        inputs.driveCurrent = driveMotor.getTorqueCurrent().getValueAsDouble();
    }

    @Override
    public void setDriveVoltage(double volts) {
        driveMotor.setVoltage(volts);
    }

    @Override
    public void setAngleVoltage(double volts) {
        angleMotor.setVoltage(volts);
    }

    private void configCANCoder(double angleOffsetDegrees) {
        CANcoderConfiguration cancoderConfigs = new CANcoderConfiguration();
        cancoderConfigs.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        cancoderConfigs.MagnetSensor.MagnetOffset = angleOffsetDegrees;
        cancoderConfigs.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

        absoluteEncoder.getConfigurator().apply(cancoderConfigs);
    }

    private void configDriveMotor(InvertedValue invertedValue) {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = invertedValue;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.CurrentLimits.StatorCurrentLimit = 60; // slip at 70
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.Audio.AllowMusicDurDisable = true;
        driveMotor.applyConfig(config);
    }

    private void configAngleMotor(boolean invertedValue) {
        // Neo is automatically reset to factory defaults upon construction
        angleMotor.setSmartCurrentLimit(MotorConstants.angleContinuousCurrentLimit);
        angleMotor.setInverted(invertedValue);
        angleMotor.setIdleMode(MotorConstants.angleNeutralMode);
        angleMotor.burnFlash();
    }

    @Override
    public void configOrchestra(Orchestra orchestra) {
        orchestra.addInstrument(driveMotor);
    }

}
