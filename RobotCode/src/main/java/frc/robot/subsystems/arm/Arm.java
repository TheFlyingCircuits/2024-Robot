// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import frc.lib.subsystem.DiagnosticSubsystem;
import frc.robot.Constants.ArmConstants;

public class Arm extends DiagnosticSubsystem {

    ArmIOInputsAutoLogged inputs;
    ArmIO io;

    private ArmFeedforward armFeedforward;
    /** PID on the arm's position. The proportional term has to do with the angle error.*/
    private ArmPDController armPD;
    private TrapezoidProfile profile;
    private Timer timer;

    private TrapezoidProfile.State initState;
    private double targetAngleDegrees;
    private double prevTargetAngleDegrees;
    private double setpointVelocityDegreesPerSecond;
    private Timer setpointVelocityTimer = new Timer();
    public boolean isMovingToTarget;

    private Mechanism2d armMech2d;
    private MechanismRoot2d armMechRoot;
    private MechanismLigament2d mechLigament;

    private SysIdRoutine.Mechanism sysIdMech;
    private SysIdRoutine sysIdRoutine;
    private boolean disableSetpointChecking = false;
    

    public Arm(ArmIO armIO) {
        io = armIO;
        inputs = new ArmIOInputsAutoLogged();

        armFeedforward = new ArmFeedforward(
            ArmConstants.kSArmVolts,
            ArmConstants.kGArmVolts,
            ArmConstants.kVArmVoltsSecondsPerRadian,
            ArmConstants.kAArmVoltsSecondsSquaredPerRadian
        );

        armPD = new ArmPDController(ArmConstants.kPArmVoltsPerDegree, ArmConstants.kDArmVoltsSecondsPerDegree);

        profile = new TrapezoidProfile(ArmConstants.constraints);
        timer = new Timer();

        isMovingToTarget = false;

        armMech2d = new Mechanism2d(1, 1);
        armMechRoot = armMech2d.getRoot("root", 0.3, 0.1);
        mechLigament = armMechRoot.append(new MechanismLigament2d("arm", 0.508, inputs.armAngleDegrees));


        
        sysIdMech = new SysIdRoutine.Mechanism(
                (Measure<Voltage> volts) -> {io.setArmMotorVolts(volts.in(Volts));},
                this::motorSysIdLog,
                this);


        sysIdRoutine = new SysIdRoutine(new Config(), sysIdMech);
    }


    //This method is in here because future commands which want to move the arm will be more easily written.
    //Rather than having to motion profile within each command, it can be just one function call.
    /**
     * Sets the desired position for the arm's motion profile to follow.
     * @param targetDegrees - Target angle in degrees for the arm.
     */
    public void setDesiredDegrees(double targetAngleDegrees) {
        // Save the last target before updating for this iteration
        prevTargetAngleDegrees = this.targetAngleDegrees;
        this.targetAngleDegrees = MathUtil.clamp(targetAngleDegrees, ArmConstants.armMinAngleDegrees, ArmConstants.armMaxAngleDegrees);
        
        double timeSinceLastSetpoint = setpointVelocityTimer.get();
        if (timeSinceLastSetpoint > 0) {
            this.setpointVelocityDegreesPerSecond = (this.targetAngleDegrees - prevTargetAngleDegrees) / timeSinceLastSetpoint;
        }
        setpointVelocityTimer.restart();
        

        //For continuous control
        if (Math.abs(this.targetAngleDegrees - prevTargetAngleDegrees) < 2) {
            // No need to generate a new profile if the requested
            // target is close to the current target. PID should get us
            // there on its own.
            return;
        }

        initState = new TrapezoidProfile.State(inputs.armAngleDegrees, inputs.armVelocityDegreesPerSecond);
        isMovingToTarget = true;
        
        timer.restart();
    }

    public Command setDesiredDegreesCommand(double targetAngleDegrees) {
        return this.run(() -> {this.setDesiredDegrees(targetAngleDegrees);});
    }

    public Command holdCurrentPositionCommand() {
        return this.run(() -> {this.setDesiredDegrees(inputs.armAngleDegrees);});
    }

    public double getDegrees() {
        return inputs.armAngleDegrees;
    }

    public double getErrorDegrees() {
        return this.targetAngleDegrees - inputs.armAngleDegrees;
    }

    public boolean isCloseToTarget() {
        // TODO: pick a non-arbitrary value based on sensor resolution?
        boolean errorIsSmall = Math.abs(getErrorDegrees()) < 1.0;
        boolean isKeepingUp = Math.abs(inputs.armVelocityDegreesPerSecond - setpointVelocityDegreesPerSecond) < 5000.0;
        return errorIsSmall && isKeepingUp;
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

            Logger.recordOutput("arm/totalOutputVolts", feedforwardOutputVolts + pidOutputVolts);

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

        totalOutputVolts = MathUtil.clamp(totalOutputVolts, -12, 12);

        if ((inputs.atLowerLimit && totalOutputVolts < 0) || (inputs.atUpperLimit && totalOutputVolts > 0))
            totalOutputVolts = 0;

        Logger.recordOutput("arm/trapezoidProfilePosition", desiredState.position);
        Logger.recordOutput("arm/totalOutputVolts", totalOutputVolts);

        io.setArmMotorVolts(totalOutputVolts);

        //profile.calculate() must be called before this line in order for isFinished() to function properly
        if (profile.isFinished(timer.get())) {
            isMovingToTarget = false;
        }
    }

    private void motorSysIdLog(SysIdRoutineLog log) {
        log.motor("leftMotor")
            .voltage(Volts.of(inputs.leftMotorAppliedVoltage))
            .angularPosition(Degrees.of(inputs.armAngleDegrees))
            .angularVelocity(DegreesPerSecond.of(inputs.armVelocityDegreesPerSecond));
        
        log.motor("rightMotor")
            .voltage(Volts.of(inputs.rightMotorAppliedVoltage))
            .angularPosition(Degrees.of(inputs.armAngleDegrees))
            .angularVelocity(DegreesPerSecond.of(inputs.armVelocityDegreesPerSecond));
    }
    
    public Command generateSysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }

    public Command generateSysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }

    public void setDisableSetpointChecking(boolean isDisabled) {
        disableSetpointChecking = isDisabled;
    }
    
    @Override
    public void periodic() {
        this.clearMotorTemps();
        this.addMotorTemps(io.getMotorTemps());
        // this.addMotorTemp(new MotorTempObject("test", 23));

        Logger.processInputs("armInputs", inputs);

        io.updateInputs(inputs);

        if(disableSetpointChecking) {
            io.setCoast(true);
        }
        else {
            io.setCoast(false);
        }

        if(!io.isCoast()) {
            followTrapezoidProfile();
        }
        else {
            io.setArmMotorVolts(0);
        }

        mechLigament.setAngle(inputs.armAngleDegrees);


        Logger.recordOutput("arm/mech2d", armMech2d);
        Logger.recordOutput("arm/isMovingToTarget", isMovingToTarget);
        Logger.recordOutput("arm/targetAngleDegrees", targetAngleDegrees);
        Logger.recordOutput("arm/isCloseToTarget", isCloseToTarget());
    }


    /** Resets the angle back to its default idle position. */
    Command resetShooterAngle() {
        double desiredAngle = ArmConstants.armMinAngleDegrees+5; // puts the arm at min height to pass under stage
        //desiredAngle = 35;
        return this.setDesiredDegreesCommand(desiredAngle);
    }

    @Override
    protected Command autoDiagnoseCommand() {
        return Commands.sequence(
            Commands.runOnce(() -> {this.setDisableSetpointChecking(false);}),
            this.setDesiredDegreesCommand(ArmConstants.armMaxAngleDegrees).withTimeout(2),
            Commands.runOnce(() -> {
                this.addFault("[Auto Diagnose] this is a test lol", false);
                if(!this.isCloseToTarget()) {
                    this.addFault("[Auto Diagnose] arm not reaching max angle", false);
                }
            }),
            this.setDesiredDegreesCommand(ArmConstants.armMinAngleDegrees).withTimeout(2),
            Commands.runOnce(() -> {
                if(!this.isCloseToTarget()) {
                    this.addFault("[Auto Diagnose] arm not reaching min angle", false);
                }
            }),
            // AMP ANGLE
            this.setDesiredDegreesCommand(110).withTimeout(2),
            Commands.runOnce(() -> {
                if(!this.isCloseToTarget()) {
                    this.addFault("[Auto Diagnose] arm not reaching AMP angle", false);
                }
            }),
            this.resetShooterAngle().withTimeout(2),
            Commands.runOnce(() -> {
                if(!this.isCloseToTarget()) {
                    this.addFault("[Auto Diagnose] arm not reaching home", false);
                }
            })
        ).andThen(this.resetShooterAngle().withTimeout(2));
    }
}