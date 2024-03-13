package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants.ShooterConstants;
import frc.robot.VendorWrappers.Kraken;

public class ShooterIOKraken implements ShooterIO {
    
    public Kraken leftMotor;
    public Kraken rightMotor;

    public ShooterIOKraken() {

        /**Left motor config */
        leftMotor = new Kraken(ShooterConstants.leftMotorID, "CTRENetwork");

        /**Right motor config */
        rightMotor = new Kraken(ShooterConstants.rightMotorID, "CTRENetwork");

        configMotors();

    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.leftFlywheelsMetersPerSecond = leftMotor.getVelocity().getValueAsDouble()*ShooterConstants.flywheelCircumferenceMeters;
        inputs.rightFlywheelsMetersPerSecond = rightMotor.getVelocity().getValueAsDouble()*ShooterConstants.flywheelCircumferenceMeters;
    
        inputs.leftMotorAppliedVoltage = leftMotor.getMotorVoltage().getValueAsDouble();
        inputs.rightMotorAppliedVoltage = rightMotor.getMotorVoltage().getValueAsDouble();

        inputs.leftMotorOutputCurrent = leftMotor.getTorqueCurrent().getValueAsDouble();
        inputs.rightMotorOutputCurrent = rightMotor.getTorqueCurrent().getValueAsDouble();
    }

    private void configMotors() {
        TalonFXConfiguration leftConfig = new TalonFXConfiguration();
        leftConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        leftConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        leftConfig.CurrentLimits.StatorCurrentLimit = 90;
        leftConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        leftMotor.applyConfig(leftConfig);

        TalonFXConfiguration rightConfig = new TalonFXConfiguration();
        rightConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        rightConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        rightConfig.CurrentLimits.StatorCurrentLimit = 90;
        rightConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        rightMotor.applyConfig(rightConfig);
    }

    @Override
    public void setLeftMotorVolts(double volts) {
        if (leftMotor.getDeviceTemp().getValueAsDouble() > ShooterConstants.motorMaxTempCelsius) {
            volts = 0;
        }

        leftMotor.setVoltage(volts);
    }

    @Override
    public void setRightMotorVolts(double volts) {
        if (rightMotor.getDeviceTemp().getValueAsDouble() > ShooterConstants.motorMaxTempCelsius) {
            volts = 0;
        }


        rightMotor.setVoltage(volts);
    }
}
