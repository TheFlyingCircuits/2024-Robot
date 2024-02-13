package frc.robot.subsystems.climb;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class Climb extends SubsystemBase{
    
    private CANSparkMax leftMotor;
    private CANSparkMax rightMotor;
    /**
     * Encoder object to get the position of the left encoder, in meters. A positive position means the arm is extended above its resting position of 0.
     */
    private RelativeEncoder leftEncoder;
    /**
     * Encoder object to get the position of the right encoder, in meters. A positive position means the arm is extended above its resting position of 0.
     */
    private RelativeEncoder rightEncoder;

    public Climb() {

        leftMotor = new CANSparkMax(ClimbConstants.leftMotorID, MotorType.kBrushless);
        rightMotor = new CANSparkMax(ClimbConstants.rightMotorID, MotorType.kBrushless);

        configMotors();

        leftEncoder = leftMotor.getEncoder();
        rightEncoder = rightMotor.getEncoder();

        configEncoders();
    }

    /**
     * @param volts - A positive value will extend the left climb arm upwards
     */
    public void setLeftMotorVolts(double volts) {
        volts = (leftEncoder.getPosition() == ClimbConstants.armMaxPosMeters) ? volts : 0;

        leftMotor.setVoltage(volts);
    }

    /**
     * @param volts - A positive value will extend the right climb arm upwards
     */
    public void setRightMotorVolts(double volts) {
        volts = (rightEncoder.getPosition() == ClimbConstants.armMaxPosMeters) ? volts : 0;

        rightMotor.setVoltage(volts);
    }

    private void configMotors() {

        //TOOD: figure out motor inversions
        leftMotor.setInverted(true);
        rightMotor.setInverted(false);

        leftMotor.setIdleMode(IdleMode.kBrake);
        rightMotor.setIdleMode(IdleMode.kBrake);

        leftMotor.burnFlash();
        rightMotor.burnFlash();

    }

    private void configEncoders() {
        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);

        leftEncoder.setPositionConversionFactor(ClimbConstants.climberArmMetersPerMotorRotation);
        rightEncoder.setPositionConversionFactor(ClimbConstants.climberArmMetersPerMotorRotation);

        leftMotor.burnFlash();
        rightMotor.burnFlash();
    }

    //TODO: add limit switches and zero encoders when hit
    public boolean climbArmsAreDown() {
        return (leftEncoder.getPosition() <= 0) && (rightEncoder.getPosition() <= 0);
    }

    public boolean climbArmsAreUp() {
        return (leftEncoder.getPosition() >= ClimbConstants.armMaxPosMeters) && (rightEncoder.getPosition() >= ClimbConstants.armMaxPosMeters);
    }
}
