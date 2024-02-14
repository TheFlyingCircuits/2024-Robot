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
        leftMotor = new TalonFX(ShooterConstants.leftMotorID);
        configMotor(leftMotor);

        /**Right motor config */
        rightMotor = new TalonFX(ShooterConstants.rightMotorID);
        configMotor(rightMotor);

    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.leftFlywheelsRotationsPerSecond = leftMotor.getVelocity().getValueAsDouble();
        inputs.rightFlywheelsRotationsPerSecond = rightMotor.getVelocity().getValueAsDouble();
    }

    private void configMotor(TalonFX motor) {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motor.getConfigurator().apply(config);
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
