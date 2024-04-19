package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
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

    public Command raiseHooksCommand() {
        return this.run(() -> {this.setVoltsClosedLoop(7);});
    }

    public Command raiseHooksCommand(double volts) {
        return this.run(() -> {this.setVoltsClosedLoop(volts);});
    }

    public Command lowerHooksCommand() {
        return this.run(() -> {this.setVoltsClosedLoop(-11);});
    }

    /**
     * @param volts - A positive value will extend the left climb arm upwards
     */
    public void setLeftMotorVolts(double volts) {
        volts = (leftArmUp() && (volts > 0)) ? 0 : volts;
        volts = (leftArmDown() && (volts < 0)) ? 0 : volts;

        leftMotor.setVoltage(volts);
    }

    /**
     * @param volts - A positive value will extend the right climb arm upwards
     */
    public void setRightMotorVolts(double volts) {
        volts = (rightArmUp() && (volts > 0)) ? 0 : volts;
        volts = (rightArmDown() && (volts < 0)) ? 0 : volts;

        rightMotor.setVoltage(volts);
    }

    /** 
     * Attempts to match position of the two climb arms using the error between them.
     * @param volts - A positive value will extend the climb arms upwards
     */
    public void setVoltsClosedLoop(double volts) {
        if (volts == 0) {
            // Secial case for safety. Don't attempt to sync
            // the arm positions if you're just asking them to both stop.
            setLeftMotorVolts(volts);
            setRightMotorVolts(volts);
            return;
        }

        double positionSyncKpVoltsPerMeter = 6;
        double leftError = rightMotor.getPosition() - leftMotor.getPosition();
        double rightError = -leftError;

        setLeftMotorVolts(volts + (positionSyncKpVoltsPerMeter * leftError));
        setRightMotorVolts(volts + (positionSyncKpVoltsPerMeter * rightError));
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

        leftMotor.setSmartCurrentLimit(65);
        rightMotor.setSmartCurrentLimit(65);

        leftMotor.setPosition(0);
        rightMotor.setPosition(0);

        leftMotor.setPositionConversionFactor(ClimbConstants.climberArmMetersPerMotorRotation);
        rightMotor.setPositionConversionFactor(ClimbConstants.climberArmMetersPerMotorRotation);

        leftMotor.burnFlash();
        rightMotor.burnFlash();

    }

    public boolean leftArmDown() {
        return leftMotor.getPosition() <= ClimbConstants.climbMinPosMeters;
    }

    public boolean rightArmDown() {
        return rightMotor.getPosition() <= ClimbConstants.climbMinPosMeters;
    }

    public boolean leftArmUp() {
        return leftMotor.getPosition() >= ClimbConstants.climbMaxPosMeters;
    }

    public boolean rightArmUp() {
        return rightMotor.getPosition() >= ClimbConstants.climbMaxPosMeters;
    }

    public boolean climbArmsZero() {
        return Math.abs(rightMotor.getPosition()) < 0.02 && Math.abs(leftMotor.getPosition()) < 0.02;
    }

    public boolean climbArmsDown() {
        return leftArmDown() && rightArmDown();
    }

    public boolean climbArmsUp() {
        return leftArmUp() && rightArmUp();
    }

    public boolean atQuickClimbSetpoint() {
        return (leftMotor.getPosition() <= 0.26) && (rightMotor.getPosition() <= 0.26);
    }
    
    @Override
    public void periodic() {
        Logger.recordOutput("climb/leftArmPositionMeters", leftMotor.getPosition());
        Logger.recordOutput("climb/rightArmPositionMeters", rightMotor.getPosition());
        Logger.recordOutput("climb/leftArmOutputCurrentAmps", leftMotor.getOutputCurrent());
        Logger.recordOutput("climb/rightArmOutputCurrentAmps", rightMotor.getOutputCurrent());
    }

}
