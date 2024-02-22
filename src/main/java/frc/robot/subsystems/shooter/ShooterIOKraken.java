package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants.ShooterConstants;

public class ShooterIOKraken implements ShooterIO {
    
    public TalonFX leftMotor;
    public TalonFX rightMotor;

    public ShooterIOKraken() {

        /**Left motor config */
        leftMotor = new TalonFX(ShooterConstants.leftMotorID, "CTRENetwork");

        /**Right motor config */
        rightMotor = new TalonFX(ShooterConstants.rightMotorID, "CTRENetwork");

        configMotors();

    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.leftFlywheelsRotationsPerSecond = leftMotor.getVelocity().getValueAsDouble();
        inputs.rightFlywheelsRotationsPerSecond = rightMotor.getVelocity().getValueAsDouble();
    }

    private void configMotors() {
        TalonFXConfiguration leftConfig = new TalonFXConfiguration();
        leftConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        leftConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        leftMotor.getConfigurator().apply(leftConfig);

        TalonFXConfiguration rightConfig = new TalonFXConfiguration();
        rightConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        rightConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        rightMotor.getConfigurator().apply(rightConfig);
    }

    @Override
    public void setLeftMotorVolts(double volts) {
        leftMotor.setVoltage(volts);
    }

    @Override
    public void setRightMotorVolts(double volts) {
        rightMotor.setVoltage(volts);
    }
}
