package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

public class SwerveModuleIOKraken implements SwerveModuleIO {
    private double angleOffsetDegrees;
    private CANcoder absoluteEncoder;
    private TalonFX angleMotor;
    private TalonFX driveMotor;

    public SwerveModuleIOKraken(int driveMotorID, int angleMotorID, int angleOffsetDegrees, int cancoderID) {
        this.angleOffsetDegrees = angleOffsetDegrees;
        
        /* Angle Encoder Config */
        absoluteEncoder = new CANcoder(cancoderID);
        configCANCoder();

        /* Angle Motor Config */
        angleMotor = new TalonFX(angleMotorID);
        configMotor(angleMotor);

        /* Drive Motor Config */
        driveMotor = new TalonFX(driveMotorID);
        configMotor(driveMotor);
    }

    @Override
    public void updateInputs(SwerveModuleIOInputs inputs) {
        inputs.drivePositionMeters = driveMotor.getPosition().getValueAsDouble();
        inputs.driveVelocityMetersPerSecond = driveMotor.getVelocity().getValueAsDouble();
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

    private void configMotor(TalonFX motor) {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motor.getConfigurator().apply(config);
    }
}
