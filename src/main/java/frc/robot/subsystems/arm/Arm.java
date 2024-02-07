// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {

    ArmIOInputsAutoLogged inputs;
    ArmIO io;

    private ArmFeedforward armFeedforward;
    /** PID on the arm's position. The proportional term has to do with the angle error.*/
    private ArmPDController armPD;
    private TrapezoidProfile profile;
    private Timer timer;

    private TrapezoidProfile.State initState;
    private double targetAngleDegrees;
    private boolean isMovingToTarget;

    public Arm(ArmIO armIO) {
        io = armIO;
        inputs = new ArmIOInputsAutoLogged();

        armFeedforward = new ArmFeedforward(
            ArmConstants.kSArmVolts,
            ArmConstants.kGArmVolts,
            ArmConstants.kVArmVoltsSecondsPerRadian,
            ArmConstants.kAArmVoltsSecondsSquaredPerRadian
        );

        armPD = new ArmPDController(0, 0);

        profile = new TrapezoidProfile(ArmConstants.constraints);
        timer = new Timer();

        isMovingToTarget = false;
    }

    /**
     * Sets the desired position for the arm's motion profile to follow.
     * @param targetDegrees - Target angle in degrees for the arm.
     */
    public void setArmDesiredPosition(double targetAngleDegrees) {
        targetAngleDegrees = MathUtil.clamp(targetAngleDegrees, ArmConstants.kArmMinAngleDegrees, ArmConstants.kArmMaxAngleDegrees);
        
        if (Math.abs(this.targetAngleDegrees - targetAngleDegrees) < 0.1)
            return;

        initState = new TrapezoidProfile.State(inputs.armAngleDegrees, inputs.armVelocityDegreesPerSecond);
        this.targetAngleDegrees = targetAngleDegrees;
        isMovingToTarget = true;
        
        timer.restart();
    }


    private void followTrapezoidProfile() {




        //Hold the current position if there's no trapezoidal profile active. 
        //I think? that generating new trapezoidal profiles for an inactive arm causes some oscillations.
        if (!isMovingToTarget) {

            double feedforwardOutputVolts = armFeedforward.calculate(
                Math.toRadians(targetAngleDegrees),
                0);
            double pidOutputVolts = armPD.calculate(
                inputs.armAngleDegrees,
                inputs.armVelocityDegreesPerSecond,
                targetAngleDegrees,
                0
            );

            io.setArmMotorVolts(feedforwardOutputVolts + pidOutputVolts);

            Logger.recordOutput("arm/postTotalOutputVolts", feedforwardOutputVolts + pidOutputVolts);

            return;
        }

        

        TrapezoidProfile.State desiredState = profile.calculate(
            timer.get(),
            initState,
            new TrapezoidProfile.State(targetAngleDegrees, 0)
        );

        double feedforwardOutputVolts = armFeedforward.calculate(
            Math.toRadians(inputs.armAngleDegrees),
            Math.toRadians(desiredState.velocity)
        );
        double pidOutputVolts = armPD.calculate(
            inputs.armAngleDegrees,
            inputs.armVelocityDegreesPerSecond,
            desiredState.position,
            desiredState.velocity
        );

        double totalOutputVolts = feedforwardOutputVolts + pidOutputVolts;

        if ((inputs.atLowerLimit && totalOutputVolts < 0) || (inputs.atUpperLimit && totalOutputVolts > 0))
            totalOutputVolts = 0;

        Logger.recordOutput("arm/trapezoidProfilePosition", desiredState.position);

        Logger.recordOutput("arm/postTotalOutputVolts", totalOutputVolts);

        io.setArmMotorVolts(totalOutputVolts);

        //profile.calculate() must be called before this line in order for isFinished() to function properly
        if (profile.isFinished(timer.get()))
            isMovingToTarget = false;
    }



    @Override
    public void periodic() {
        
        io.updateInputs(inputs);

        followTrapezoidProfile();

        Logger.processInputs("Arm", inputs);

        Logger.recordOutput("arm/isMovingToTarget", isMovingToTarget);
        Logger.recordOutput("arm/targetAngleDegrees", targetAngleDegrees);
    }
}
