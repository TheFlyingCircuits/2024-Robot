package frc.robot.subsystems.arm;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.MotorConstants;
import frc.robot.subsystems.arm.ArmIO.ArmIOInputs;

public class ArmIONeo implements ArmIO {
    
    private CANSparkMax leftMotor;
    private CANSparkMax rightMotor;
    private CANcoder leftAbsoluteEncoder;
    private CANcoder rightAbsoluteEncoder;

    public ArmIONeo() {

        /**CANcoder config */
        leftAbsoluteEncoder = new CANcoder(ArmConstants.leftArmCANcoderID, "CTRENetwork");
        rightAbsoluteEncoder = new CANcoder(ArmConstants.rightArmCANcoderID, "CTRENetwork");
        configCANCoders();

        /** Motor config */
        leftMotor = new CANSparkMax(ArmConstants.leftMotorID, MotorType.kBrushless);
        rightMotor = new CANSparkMax(ArmConstants.rightMotorID, MotorType.kBrushless);
        
        configMotors();

    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        //TODO: fuse both cancoder measurements
        inputs.armVelocityDegreesPerSecond = rightAbsoluteEncoder.getVelocity().getValueAsDouble() * 360;
        inputs.armAngleDegrees = rightAbsoluteEncoder.getAbsolutePosition().getValueAsDouble()*360;
    }

    @Override
    public void setArmMotorVolts(double volts) {
        leftMotor.setVoltage(volts);
        rightMotor.setVoltage(volts);
    }

    private void configMotors() {
        rightMotor.restoreFactoryDefaults();
        rightMotor.setSmartCurrentLimit(MotorConstants.angleContinuousCurrentLimit);
        rightMotor.setInverted(false);
        rightMotor.setIdleMode(IdleMode.kBrake);
        rightMotor.burnFlash();

        leftMotor.restoreFactoryDefaults();
        leftMotor.setSmartCurrentLimit(MotorConstants.angleContinuousCurrentLimit);
        leftMotor.setInverted(true);
        leftMotor.setIdleMode(IdleMode.kBrake);
        leftMotor.burnFlash();
    }

    private void configCANCoders() {
        CANcoderConfiguration leftCANCoderConfig = new CANcoderConfiguration();
        leftCANCoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        leftCANCoderConfig.MagnetSensor.MagnetOffset = ArmConstants.leftArmCANcoderOffset;
        leftCANCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;

        leftAbsoluteEncoder.getConfigurator().apply(leftCANCoderConfig);

        CANcoderConfiguration rightCANCoderConfig = new CANcoderConfiguration();
        rightCANCoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        rightCANCoderConfig.MagnetSensor.MagnetOffset = ArmConstants.rightArmCANcoderOffset;
        rightCANCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;        

        rightAbsoluteEncoder.getConfigurator().apply(rightCANCoderConfig);
    }

}
