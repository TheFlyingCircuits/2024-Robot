package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;
import frc.robot.VendorWrappers.Neo;

public class Climb extends SubsystemBase{
    
    private Neo leftMotor;
    private Neo rightMotor;

    public Climb() {

        leftMotor = new Neo("leftClimb", ClimbConstants.leftMotorID);
        rightMotor = new Neo("rightClimb", ClimbConstants.rightMotorID);

        configMotors();
    }

    /**
     * @param volts - A positive value will extend the left climb arm upwards
     */
    public void setLeftMotorVolts(double volts) {
        volts = (climbArmsAreUp() && (volts > 0)) ? 0 : volts;
        volts = (climbArmsAreDown() && (volts < 0)) ? 0 : volts;

        leftMotor.setVoltage(volts);
    }

    /**
     * @param volts - A positive value will extend the right climb arm upwards
     */
    public void setRightMotorVolts(double volts) {
        volts = (climbArmsAreUp() && (volts > 0)) ? 0 : volts;
        volts = (climbArmsAreDown() && (volts < 0)) ? 0 : volts;

        rightMotor.setVoltage(volts);
    }

    /**
     * @param volts - A positive value will extend the climb arms upwards
     */
    public void setVolts(double volts) {
        setLeftMotorVolts(volts);
        setRightMotorVolts(volts);
    }

    private void configMotors() {

        leftMotor.setInverted(true);
        rightMotor.setInverted(false);

        leftMotor.setIdleMode(IdleMode.kBrake);
        rightMotor.setIdleMode(IdleMode.kBrake);

        leftMotor.setSmartCurrentLimit(45);
        rightMotor.setSmartCurrentLimit(45);

        leftMotor.setPosition(0);
        rightMotor.setPosition(0);

        leftMotor.setPositionConversionFactor(ClimbConstants.climberArmMetersPerMotorRotation);
        rightMotor.setPositionConversionFactor(ClimbConstants.climberArmMetersPerMotorRotation);

        leftMotor.burnFlash();
        rightMotor.burnFlash();

    }

    public boolean climbArmsAreDown() {
        return (leftMotor.getPosition() <= ClimbConstants.climbMinPoseMeters) || (rightMotor.getPosition() <= ClimbConstants.climbMinPoseMeters);
    }

    public boolean climbArmsAreUp() {
        return (leftMotor.getPosition() >= ClimbConstants.climbMaxPosMeters) || (rightMotor.getPosition() >= ClimbConstants.climbMaxPosMeters);
    }
    
    @Override
    public void periodic() {
        Logger.recordOutput("climb/leftArmPositionMeters", leftMotor.getPosition());
        Logger.recordOutput("climb/rightArmPositionMeters", rightMotor.getPosition());
        Logger.recordOutput("climb/leftArmOutputCurrentAmps", leftMotor.getOutputCurrent());
        Logger.recordOutput("climb/rightArmOutputCurrentAmps", rightMotor.getOutputCurrent());
    }

}
