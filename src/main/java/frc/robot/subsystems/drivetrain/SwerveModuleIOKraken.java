package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.SwerveModuleConstants;

public class SwerveModuleIOKraken implements SwerveModuleIO {
    private double angleOffsetDegrees;
    private CANcoder absoluteEncoder;
    private CANSparkMax angleMotor;
    private TalonFX driveMotor;
    private RelativeEncoder angleEncoder;
    /**
     * 
     * @param driveMotorID - ID of the drive motor
     * @param angleMotorID - ID of the angle motor
     * @param angleOffsetDegrees - Offset of the angle motor, in degrees
     * @param cancoderID - ID of the absolute CANcoder mounted ontop of the swerve
     * @param isDriveMotorOnTop - Is drive motor mounted on top
     * @param isAngleMotorOnTop - Is angle motor mounted on top
     */
    public SwerveModuleIOKraken(int driveMotorID, int angleMotorID, int angleOffsetDegrees, int cancoderID, boolean isDriveMotorOnTop, boolean isAngleMotorOnTop){
        this.angleOffsetDegrees = angleOffsetDegrees;
        
        /* Angle Encoder Config */
        absoluteEncoder = new CANcoder(cancoderID);
        configCANCoder();

        /* Angle Motor Config */
        angleMotor = new CANSparkMax(angleMotorID, MotorType.kBrushless);
        angleEncoder = angleMotor.getEncoder();
        if(isAngleMotorOnTop) {
            configAngleMotor(false);
        } else {
            configAngleMotor(true);
        }

        /* Drive Motor Config */
        driveMotor = new TalonFX(driveMotorID);
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
    }

    @Override
    public void setDriveVoltage(double volts) {
        driveMotor.setVoltage(volts);
    }

    @Override
    public void setAngleVoltage(double volts) {
        angleMotor.setVoltage(volts);
    }

    private void configCANCoder() {
        CANcoderConfiguration cancoderConfigs = new CANcoderConfiguration();
        cancoderConfigs.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        cancoderConfigs.MagnetSensor.MagnetOffset = angleOffsetDegrees;
        cancoderConfigs.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

        absoluteEncoder.getConfigurator().apply(cancoderConfigs);
    }

    //TODO: angle motor is neo, drive motor is kraken

    private void configDriveMotor(InvertedValue invertedValue) {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = invertedValue;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        driveMotor.getConfigurator().apply(config);
    }

    private void configAngleMotor(boolean invertedValue) {
        angleMotor.restoreFactoryDefaults();
        angleMotor.setSmartCurrentLimit(MotorConstants.angleContinuousCurrentLimit);
        angleMotor.setInverted(invertedValue);
        angleMotor.setIdleMode(MotorConstants.angleNeutralMode);
        //converts rotations of motor into deg of wheel
        angleEncoder.setPositionConversionFactor(SwerveModuleConstants.steerGearReduction*360.0);
        //converts rpm of motor into deg/s of wheel
        angleEncoder.setVelocityConversionFactor(SwerveModuleConstants.steerGearReduction*360.0/60.0);
        angleMotor.burnFlash();
    }

}
