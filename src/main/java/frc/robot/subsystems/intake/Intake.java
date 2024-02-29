// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.MotorConstants;
import frc.robot.VendorWrappers.Neo;

/** Add your docs here. */
public class Intake extends SubsystemBase {

    private DigitalInput intakeProximitySwitch;
    /**
     * Motor object that controls the four axles on the front of the intake. 
     * A positive voltage spins the axles to suck a note into the robot.
     */
    private Neo frontIntakeMotor;
    /**
     * Motor object that controls the two axles on the back of the intake.
     * A positive voltage spins the axles to suck a note into the robot.
     */
    private Neo backIntakeMotor;

    public Intake() {
        intakeProximitySwitch = new DigitalInput(IntakeConstants.intakeProximitySwitchID);

        frontIntakeMotor = new Neo("frontIntake", IntakeConstants.frontIntakeMotorID);
        backIntakeMotor = new Neo("backIntake", IntakeConstants.backIntakeMotorID);
        
        configMotors();
    }

    public boolean isRingInIntake() {
        return !intakeProximitySwitch.get();
    }

    /**
     * Sets the voltage of both intake motors.
     * @param volts - Voltage to feed the intake motors. A positive value will intake a note.
     */
    public void setVolts(double volts) {
        frontIntakeMotor.setVoltage(volts);
        backIntakeMotor.setVoltage(volts);
    }

    private void configMotors() {
        frontIntakeMotor.setInverted(false);
        backIntakeMotor.setInverted(false);

        frontIntakeMotor.setSmartCurrentLimit(50);
        backIntakeMotor.setSmartCurrentLimit(50);

        frontIntakeMotor.setIdleMode(IdleMode.kBrake);
        backIntakeMotor.setIdleMode(IdleMode.kBrake);

        frontIntakeMotor.setVelocityConversionFactor(1);
        backIntakeMotor.setVelocityConversionFactor(1);

        frontIntakeMotor.burnFlash();
        backIntakeMotor.burnFlash();
    }


    @Override
    public void periodic() {
        Logger.recordOutput("intake/frontIntakeMotorRPM", frontIntakeMotor.getVelocity());
        Logger.recordOutput("intake/backIntakeMotorRPM", backIntakeMotor.getVelocity());
        Logger.recordOutput("intake/isRingInIntake", isRingInIntake());
    }

}
