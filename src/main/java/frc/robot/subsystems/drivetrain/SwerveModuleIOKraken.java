package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

public class SwerveModuleIOKraken implements SwerveModuleIO {
    private double angleOffsetDegrees;
    private CANcoder absoluteEncoder;
    private TalonFX angleMotor;
    private TalonFX driveMotor;


    //TODO: THIS CLASS IS STILL A WORK IN PROGRESS. DO NOT CONSIDER IT DONE.
    public SwerveModuleIOKraken(int driveMotorID, int angleMotorID, int angleOffsetDegrees, int cancoderID){
        this.angleOffsetDegrees = angleOffsetDegrees;
        
        /* Angle Encoder Config */
        absoluteEncoder = new CANcoder(cancoderID);
        configCANCoder();

        /* Angle Motor Config */
        angleMotor = new TalonFX(angleMotorID);

        /* Drive Motor Config */
        driveMotor = new TalonFX(driveMotorID);
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
        CANcoderConfiguration cancoderConfigs = new CANcoderConfiguration(); //idk if this is how youre supposed to do this but its kinda packaged with talonFX sooo
        cancoderConfigs.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf; // last config had [-180, +180) but this only has [-0.5, +0.5) as an option
        cancoderConfigs.MagnetSensor.MagnetOffset = angleOffsetDegrees;
        cancoderConfigs.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

        absoluteEncoder.getConfigurator().apply(cancoderConfigs);
    }

    private void configAngleMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        
    }
}
