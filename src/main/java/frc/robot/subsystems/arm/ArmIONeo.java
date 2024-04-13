package frc.robot.subsystems.arm;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkBase.FaultID;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.filter.LinearFilter;
import frc.robot.Constants.ArmConstants;
import frc.robot.VendorWrappers.Neo;
import frc.robot.subsystems.arm.ArmIO.ArmIOInputs;

public class ArmIONeo implements ArmIO {
    


    private LinearFilter velocityFilter;

    private Neo leftMotor;
    private Neo rightMotor;
    private CANcoder leftAbsoluteEncoder;
    private CANcoder rightAbsoluteEncoder;
    private boolean isCoast = false;

    public ArmIONeo() {

        /**CANcoder config */
        leftAbsoluteEncoder = new CANcoder(ArmConstants.leftArmCANcoderID, "CTRENetwork");
        rightAbsoluteEncoder = new CANcoder(ArmConstants.rightArmCANcoderID, "CTRENetwork");
        configCANCoders();

        /** Motor config */
        leftMotor = new Neo("leftPivot", ArmConstants.leftMotorID);
        rightMotor = new Neo("rightPivot", ArmConstants.rightMotorID);
        
        configMotors();

        velocityFilter = LinearFilter.movingAverage(8);

    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        
        inputs.armVelocityDegreesPerSecond = velocityFilter.calculate(
            rightAbsoluteEncoder.getVelocity().getValueAsDouble()*360);

        inputs.leftEncoderReadingDegrees = leftAbsoluteEncoder.getAbsolutePosition().getValueAsDouble()*360;
        inputs.rightEncoderReadingDegrees = rightAbsoluteEncoder.getAbsolutePosition().getValueAsDouble()*360;

        if (rightAbsoluteEncoder.getAbsolutePosition().getStatus().isOK()) {
            inputs.armAngleDegrees = inputs.rightEncoderReadingDegrees;
        }
        else if (leftAbsoluteEncoder.getAbsolutePosition().getStatus().isOK()) {
            inputs.armAngleDegrees = inputs.leftEncoderReadingDegrees;
        }
    
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

        inputs.leftMotorAmps = leftMotor.getOutputCurrent();
        inputs.rightMotorAmps = rightMotor.getOutputCurrent();


    }

    @Override
    public void setArmMotorVolts(double volts) {
        leftMotor.setVoltage(volts);
        rightMotor.setVoltage(volts);
    }
    
    @Override
    public void setArmEncoderPosition(double degrees) {
        leftAbsoluteEncoder.setPosition(degrees/360.);
        rightAbsoluteEncoder.setPosition(degrees/360.);
    }

    private void configMotors() {
        int ampLimit = 30;
        rightMotor.restoreFactoryDefaults();
        rightMotor.setSmartCurrentLimit(ampLimit);
        rightMotor.setInverted(false);
        rightMotor.setIdleMode(IdleMode.kBrake);
        rightMotor.burnFlash();

        leftMotor.restoreFactoryDefaults();
        leftMotor.setSmartCurrentLimit(ampLimit);
        leftMotor.setInverted(true);
        leftMotor.setIdleMode(IdleMode.kBrake);
        leftMotor.burnFlash();
    }

    @Override
    public void setCoast(boolean makeCoast) {
        if(makeCoast && !isCoast) {
            rightMotor.setIdleMode(IdleMode.kCoast);
            leftMotor.setIdleMode(IdleMode.kCoast);
            isCoast = true;
        }
        else if ((!makeCoast) && isCoast) {
            rightMotor.setIdleMode(IdleMode.kBrake);
            leftMotor.setIdleMode(IdleMode.kBrake);
            isCoast = false;
        }
    }
    @Override
    public boolean isCoast() {
        return isCoast;
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
