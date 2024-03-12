// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
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

    private SimpleMotorFeedforward frontFeedforward;
    private SimpleMotorFeedforward backFeedforward;
    
    private PIDController frontController;
    private PIDController backController;

    public Intake() {
        intakeProximitySwitch = new DigitalInput(IntakeConstants.intakeProximitySwitchID);

        frontIntakeMotor = new Neo("frontIntake", IntakeConstants.frontIntakeMotorID);
        backIntakeMotor = new Neo("backIntake", IntakeConstants.backIntakeMotorID);

        frontFeedforward = new SimpleMotorFeedforward(
            IntakeConstants.kSFrontIntakeVolts,
            IntakeConstants.kVFrontIntakeVoltsPerRPM);

        backFeedforward = new SimpleMotorFeedforward(
            IntakeConstants.kSBackIntakeVolts, 
            IntakeConstants.kVBackIntakeVoltsPerRPM);

        frontController = new PIDController(
            IntakeConstants.kPFrontIntakeVoltsPerRPM, 0, 0);

        backController = new PIDController(
            IntakeConstants.kPBackIntakeVoltsPerRPM, 0, 0);
        
        configMotors();
    }

    public boolean isRingInIntake() {
        // Proximit sensor pulls the digital input pin high by default,
        // and pulls it low when it detects an object.
        return !intakeProximitySwitch.get();
    }

    /**
     * Sets the RPM of the intake axles, using closed loop control.
     * @param rpm - RPM to have the motors spin at. A positive value will intake the note.
     */
    public void setRPM(double rpm) {
        double frontFeedforwardOutputVolts = frontFeedforward.calculate(rpm);
        double frontPIDOutputVolts = frontController.calculate(getFrontRPM(), rpm);
        frontIntakeMotor.setVoltage(frontFeedforwardOutputVolts + frontPIDOutputVolts);

        double backFeedforwardOutputVolts = backFeedforward.calculate(rpm);
        double backPIDOutputVolts = backController.calculate(getBackRPM(), rpm);
        backIntakeMotor.setVoltage(backFeedforwardOutputVolts + backPIDOutputVolts);

    }

    /**
     * Sets the voltage of both intake motors.
     * @param volts - Voltage to feed the intake motors. A positive value will intake a note.
     */
    public void setVolts(double volts) {
        frontIntakeMotor.setVoltage(volts);
        backIntakeMotor.setVoltage(volts);
    }

    public Command setVoltsCommand(double volts) {
        return this.run(() -> {this.setVolts(volts);});
    }

    public double getFrontRPM() {
        return frontIntakeMotor.getVelocity();
    }

    public double getBackRPM() {
        return backIntakeMotor.getVelocity();
    }

    private void configMotors() {
        frontIntakeMotor.setInverted(false);
        backIntakeMotor.setInverted(false);

        frontIntakeMotor.setSmartCurrentLimit(60);
        backIntakeMotor.setSmartCurrentLimit(60);

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
