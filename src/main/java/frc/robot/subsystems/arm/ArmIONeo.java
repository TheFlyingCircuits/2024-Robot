package frc.robot.subsystems.arm;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.SwerveModuleConstants;
import frc.robot.subsystems.arm.ArmIO.ArmIOInputs;

public class ArmIONeo implements ArmIO {
    
    private CANSparkMax leftMotor;
    private CANSparkMax rightMotor;
    private CANcoder absoluteEncoder;
    private double angleOffset;
    public ArmIONeo(double angleOffset) {
        this.angleOffset = angleOffset;

        /**CANcoder config */
        absoluteEncoder = new CANcoder(ArmConstants.armCANcoderID);
        configCANCoder();

        /** Left motor config */
        leftMotor = new CANSparkMax(ArmConstants.leftMotorID, MotorType.kBrushless);
        configLeftMotor();

        /** Right motor config */
        rightMotor = new CANSparkMax(ArmConstants.rightMotorID, MotorType.kBrushless);
        configRightMotor();

    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        inputs.armVelocityDegreesPerSecond = absoluteEncoder.getVelocity().getValueAsDouble() * 360;
        inputs.armAngleDegrees = absoluteEncoder.getAbsolutePosition().getValueAsDouble()*360;
    }

    public void setLeftMotorVolts(double volts) {
        leftMotor.setVoltage(volts);
    }

    public void setRightMotorVolts(double volts) {
        rightMotor.setVoltage(volts);
    }

    private void configRightMotor() {
        rightMotor.restoreFactoryDefaults();
        rightMotor.setSmartCurrentLimit(SwerveModuleConstants.angleContinuousCurrentLimit);
        rightMotor.setInverted(SwerveModuleConstants.angleInvert);
        rightMotor.setIdleMode(SwerveModuleConstants.angleNeutralMode);
        rightMotor.burnFlash();
    }

    private void configLeftMotor() {
        leftMotor.restoreFactoryDefaults();
        leftMotor.setSmartCurrentLimit(SwerveModuleConstants.angleContinuousCurrentLimit);
        leftMotor.setInverted(false);
        leftMotor.setIdleMode(SwerveModuleConstants.angleNeutralMode);
        leftMotor.burnFlash();
    }

    private void configCANCoder() {
        CANcoderConfiguration cancoderConfigs = new CANcoderConfiguration();
        cancoderConfigs.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        cancoderConfigs.MagnetSensor.MagnetOffset = angleOffset;
        cancoderConfigs.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

        absoluteEncoder.getConfigurator().apply(cancoderConfigs);
    }

}
