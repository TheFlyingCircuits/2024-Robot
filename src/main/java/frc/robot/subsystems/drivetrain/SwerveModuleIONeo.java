package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.SwerveModuleConstants;
import frc.robot.subsystems.drivetrain.SwerveModuleIO.SwerveModuleIOInputs;

public class SwerveModuleIONeo implements SwerveModuleIO{

    /**
     * This angle offset is added directly to the raw angle read by the absolute encoder.
     */
    private double angleOffset;
    private CANcoder absoluteEncoder;
    private CANSparkMax angleMotor;
    private CANSparkMax driveMotor;
    private RelativeEncoder angleEncoder;
    private RelativeEncoder driveEncoder;


    /**
     * Constructs the hardware implementation for each swerve module
     * @param driveMotorID - ID of the motor controller to the drive motor 
     * @param angleMotorID - ID of the motor controller to the angle motor
     * @param angleOffset - Offset for the individual CANcoders on each swerve module, in +-1
     * @param cancoderID - ID of the CANcoders mounted on each swerve module
     */
    public SwerveModuleIONeo(int driveMotorID, int angleMotorID, double angleOffset, int cancoderID){
        this.angleOffset = angleOffset;
        
        /* Angle Encoder Config */
        absoluteEncoder = new CANcoder(cancoderID);
        configCANCoder();

        /** Angle motor config */
        angleMotor = new CANSparkMax(angleMotorID, MotorType.kBrushless);
        angleEncoder = angleMotor.getEncoder();
        configAngleMotor();

        /** Drive motor config */
        driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
        driveEncoder = driveMotor.getEncoder();
        configDriveMotor();

    }

    @Override
    public void updateInputs(SwerveModuleIOInputs inputs) {
        inputs.drivePositionMeters = driveEncoder.getPosition();
        inputs.driveVelocityMetersPerSecond = driveEncoder.getVelocity();
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
        cancoderConfigs.MagnetSensor.MagnetOffset = angleOffset;
        cancoderConfigs.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

        absoluteEncoder.getConfigurator().apply(cancoderConfigs);
    }

    private void configAngleMotor() {
        angleMotor.restoreFactoryDefaults();
        angleMotor.setSmartCurrentLimit(MotorConstants.angleContinuousCurrentLimit);
        angleMotor.setInverted(MotorConstants.angleInvert);
        angleMotor.setIdleMode(MotorConstants.angleNeutralMode);
        //converts rotations of motor into deg of wheel
        angleEncoder.setPositionConversionFactor(SwerveModuleConstants.steerGearReduction*360.0);
        //converts rpm of motor into deg/s of wheel
        angleEncoder.setVelocityConversionFactor(SwerveModuleConstants.steerGearReduction*360.0/60.0);
        angleMotor.burnFlash();
    }

    private void configDriveMotor() {
        driveMotor.restoreFactoryDefaults();
        driveMotor.setSmartCurrentLimit(MotorConstants.driveContinuousCurrentLimit);
        driveMotor.setInverted(MotorConstants.driveInvert);
        driveMotor.setIdleMode(MotorConstants.driveNeutralMode);
        
        driveEncoder.setPosition(0.0);

        // Converts rotations of motor to meters traveled of wheel.
        driveEncoder.setPositionConversionFactor(
            SwerveModuleConstants.driveGearReduction
            * SwerveModuleConstants.wheelCircumferenceMeters
        );

        // Converts rpm of motor to m/s of wheel.
        driveEncoder.setVelocityConversionFactor(
            1./60. * SwerveModuleConstants.driveGearReduction
            * SwerveModuleConstants.wheelCircumferenceMeters
        );

        REVLibError err = driveEncoder.setMeasurementPeriod(10);
        if (err == REVLibError.kOk) {
            System.out.println("successfully set drive encoder measurement period");
        }
        
        err = driveEncoder.setAverageDepth(2);
        if (err == REVLibError.kOk) {
            System.out.println("successfully set drive encoder window size");
        }

        driveMotor.burnFlash();
    }
}
