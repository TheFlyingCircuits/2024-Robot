// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import frc.robot.Constants.ArmConstants;
import frc.robot.spring_controller_stuff.KinematicsTracker;
import frc.robot.spring_controller_stuff.SpringController;

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

    private SpringController spring = new SpringController("arm");
    private int springMovingWindowSize = 9;
    private KinematicsTracker armSetpointKinematics = new KinematicsTracker(Math.toRadians(ArmConstants.armMinAngleDegrees), springMovingWindowSize);
    private KinematicsTracker armKinematics = new KinematicsTracker(Math.toRadians(ArmConstants.armMinAngleDegrees), springMovingWindowSize);
    

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
        boolean isKeepingUp = Math.abs(inputs.armVelocityDegreesPerSecond - setpointVelocityDegreesPerSecond) < 1.0;
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


            double maxDesiredNetTorque = ArmConstants.momentOfInertiaKgMSquared * ArmConstants.armMaxAccelDegreesPerSecondSquared;
            //magnitude of max desired voltage when moving the arm downwards
            double maxDesiredTorqueDown = maxDesiredNetTorque - getTorqueFromGravity();
            
            
            
            //magnitude of max desired voltage when moving the arm upwards
            double maxDesiredTorqueUp = maxDesiredNetTorque + getTorqueFromGravity();

            // double outputVolts = MathUtil.clamp(
            //     feedforwardOutputVolts + pidOutputVolts,
            //     -maxDesiredVoltageDown,
            //     maxDesiredVoltageUp);

            double outputVolts = feedforwardOutputVolts + pidOutputVolts;

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

    public Command setCoastCommand(boolean shouldCoast) {
        return new InstantCommand(() -> {disableSetpointChecking = shouldCoast;});
    }

    public void setVolts(double volts) {
        io.setArmMotorVolts(volts);
    }

    private double getTorqueFromGravity() {
        // Offset the torque from gravity
        double torqueFromGravityWhenLevel = 20.22475; // emperical value from spring test (seems a little high)
        // torqueFromGravityWhenLevel = 16.61111; // 0.25 volt (emperical value from voltage test)
        // torqueFromGravityWhenLevel = 6.64444; // 0.1 volt (lowest before drop)
        // highest before lift was 0.4 volts. middle was (0.4 + 0.1) / 2 = 0.25 volts
        // torqueFromGravityWhenLevel = 11;
        return torqueFromGravityWhenLevel * Math.cos(armKinematics.position);
        
    }
    
    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("armInputs", inputs);

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

    public void exertTorque(double newtonMeters) {
        double torqueFromMotors = newtonMeters / ArmConstants.armGearReduction;
        io.setArmMotorTorque(torqueFromMotors);
    }


    public void testSpringControl(boolean moveMotors) {
        double momentOfInertia = (5. / 0.0919976); // Ben thinks this doesn't pass the smell test?
                                                   // update, I agree. CAD and rod approximation
                                                   // claim the MOI should be ~0.8, while this
                                                   // yeilds an MOI of ~54, which seems like
                                                   // way too big of a discrepency to account for
                                                   // the extra mass from gears/pulleys.

        momentOfInertia = ArmConstants.momentOfInertiaKgMSquared;

        double desiredDegrees = SmartDashboard.getNumber("armDesiredDegrees", ArmConstants.armMinAngleDegrees);
        SmartDashboard.putNumber("armDesiredDegrees", desiredDegrees);
        armSetpointKinematics.update(Math.toRadians(desiredDegrees));
        armKinematics.update(Math.toRadians(getDegrees()));

        double accel = spring.getDesiredAccel(armKinematics, armSetpointKinematics, momentOfInertia);
        double torque = accel * momentOfInertia;

        torque += getTorqueFromGravity(); // TODO: signs are technically wrong.

        // TESTING / TUNING
        // torque = getTorqueFromGravity();
        // torque += 5.;

        int windowSize = (int) SmartDashboard.getNumber("windowSize", 16);
        if (windowSize != this.springMovingWindowSize) {
            armKinematics = new KinematicsTracker(Math.toRadians(getDegrees()), windowSize);
            armSetpointKinematics = new KinematicsTracker(Math.toRadians(desiredDegrees), windowSize);
            this.springMovingWindowSize = windowSize;
            // 25 = is too big and has too much delay.
            // I know this because the system becomes under-damped
            //
            // spring constant 400 with 16 window size seems ok?
            // overshoot when changing the setpoint though...
            // maybe spring of 100 with window of 16 is safer?
        }
        SmartDashboard.putNumber("windowSize", this.springMovingWindowSize);


        // TODO: add compensation torque to overcome stiction?


        if (!moveMotors) {
            this.setVolts(0);
            return;
        }
        this.exertTorque(torque);
    }

    public Command runSpringControl(boolean moveMotors) {
        return this.run(() -> {this.testSpringControl(moveMotors);});
    }
}
