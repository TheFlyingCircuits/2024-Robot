package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;

public class SwerveModuleIOKraken implements SwerveModuleIO {
    private double angleOffset;
    private CANcoder absoluteEncoder;
    private TalonFX mAngleMotor;
    private TalonFX mDriveMotor;
    private TalonFXConfiguration MotorConfigs; // K-stuff in docs ben said no no
    private CANcoderConfiguration CANConfigs;

    public SwerveModuleIOKraken(int driveMotorID, int angleMotorID, int angleOffset, int cancoderID){
        this.angleOffset = angleOffset;
        
        /* Angle Encoder Config */
        absoluteEncoder = new CANcoder(cancoderID);
        configCANCoder();
        /* Angle Motor Config */
        mAngleMotor = new TalonFX(angleMotorID);

        /* Drive Motor Config */
        mDriveMotor = new TalonFX(driveMotorID);
    }

    @Override
    public void updateInputs(SwerveModuleIOInputs inputs) {
        inputs.drivePositionMeters = mDriveMotor.getPosition().getValueAsDouble();
        inputs.driveVelocityMetersPerSecond = mDriveMotor.getVelocity().getValueAsDouble();
        inputs.angleAbsolutePositionDegrees = absoluteEncoder.getAbsolutePosition().getValueAsDouble();
    }

    @Override
    public void setDriveVoltage(double volts) {
        mDriveMotor.setVoltage(volts);
    }

    @Override
    public void setAngleVoltage(double volts) {
        mAngleMotor.setVoltage(volts);
    }

    private void configCANCoder() {
        CANConfigs = new CANcoderConfiguration(); //idk if this is how youre supposed to do this but its kinda packaged with talonFX sooo
        CANConfigs.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf; // last config had [-180, +180) but this only has [-0.5, +0.5) as an option
        CANConfigs.MagnetSensor.MagnetOffset = angleOffset;
        absoluteEncoder.getConfigurator().apply(CANConfigs);
    }
    
}
