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
        inputs.armAngleDegrees = rightAbsoluteEncoder.getPosition().getValueAsDouble()*360;

        //getBusVoltage() gets voltage fed into motor controller, getAppliedOutput() gets a percent the motor is running at.
        //found this solution on chiefdelphi
        inputs.leftMotorAppliedVoltage = leftMotor.getAppliedOutput()*leftMotor.getBusVoltage();
        inputs.rightMotorAppliedVoltage = rightMotor.getAppliedOutput()*rightMotor.getBusVoltage();

        if (inputs.armAngleDegrees <= ArmConstants.armMinAngleDegrees) {inputs.atLowerLimit = true;}
        else if (inputs.armAngleDegrees >= ArmConstants.armMaxAngleDegrees) {inputs.atUpperLimit = true;}
        else {
            inputs.atLowerLimit = false;
            inputs.atUpperLimit = false;
        }
    }

    @Override
    public void setArmMotorVolts(double volts) {
        leftMotor.setVoltage(volts);
        rightMotor.setVoltage(volts);
    }
    
    @Override
    public void setArmEncoderPosition(double degrees) {
        rightAbsoluteEncoder.setPosition(degrees/360.);
    }

    private void configMotors() {
        rightMotor.restoreFactoryDefaults();
        rightMotor.setSmartCurrentLimit(80);
        rightMotor.setInverted(false);
        rightMotor.setIdleMode(IdleMode.kBrake);
        rightMotor.burnFlash();

        leftMotor.restoreFactoryDefaults();
        leftMotor.setSmartCurrentLimit(80);
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
