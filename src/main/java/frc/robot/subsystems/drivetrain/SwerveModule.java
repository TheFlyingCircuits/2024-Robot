package frc.robot.subsystems.drivetrain;

import frc.robot.Constants.SwerveModuleConstants;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;


public class SwerveModule {

    public int moduleIndex;

    private SwerveModuleIO io;
    private SwerveModuleIOInputsAutoLogged inputs;

    private static PIDController drivePID;
    private static PIDController anglePID;
    private SimpleMotorFeedforward driveFeedforward;

    public SwerveModule(SwerveModuleIO io, int moduleIndex) {
        this.io = io;
        this.moduleIndex = moduleIndex;

        inputs = new SwerveModuleIOInputsAutoLogged();
        
        drivePID = new PIDController(
            SwerveModuleConstants.drivekPVoltsPerMeterPerSecond, 
            SwerveModuleConstants.drivekIVoltsPerMeter, 
            SwerveModuleConstants.drivekDVoltsPerMeterPerSecondSquared);
        anglePID = new PIDController(
            SwerveModuleConstants.anglekPVoltsPerDegree,
            SwerveModuleConstants.anglekIVoltsPerDegreeSeconds,
            SwerveModuleConstants.anglekDVoltsPerDegreePerSecond);

        driveFeedforward = new SimpleMotorFeedforward(
            SwerveModuleConstants.drivekSVolts, 
            SwerveModuleConstants.drivekVVoltsSecondsPerMeter, 
            SwerveModuleConstants.drivekAVoltsSecondsSquaredPerMeter);

        anglePID.enableContinuousInput(-180, 180);
    }

    public void periodic() {
        io.updateInputs(inputs);
    }


    private SwerveModuleState constrainState(SwerveModuleState toConstrain) {
        double originalDegrees = toConstrain.angle.getDegrees();
        double constrainedDegrees = MathUtil.inputModulus(originalDegrees, -180, 180);
        return new SwerveModuleState(toConstrain.speedMetersPerSecond, Rotation2d.fromDegrees(constrainedDegrees));
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean closedLoop) {
        // Make sure the desired state's angle is in the range [-180, 180], so it can be appropriately
        // compared to the current angle from the CANCoder. We verified that you need constrainState()
        // to be called both before and after SwerveModuleState.optimize().
        desiredState = constrainState(desiredState);
        desiredState = SwerveModuleState.optimize(
            desiredState,
            Rotation2d.fromDegrees(inputs.angleAbsolutePositionDegrees)
        );

                
        
        // Logger.recordOutput("desiredState", desiredState);
        setDesiredStateNoOptimize(desiredState, closedLoop);
    }

    public void setDesiredStateNoOptimize(SwerveModuleState desiredState, boolean closedLoop) {
        
        desiredState = constrainState(desiredState); // constrain one more time after optimization just to be safe, because I'm unsure if optimization can ever pull the angle out of [-180, 180]


        if (closedLoop) {
        // Conversion factor is already set below to convert rpm of motor to m/s of wheel.
            double wheelMetersPerSecond = inputs.driveVelocityMetersPerSecond;

            double feedforward = driveFeedforward.calculate(desiredState.speedMetersPerSecond);
            double pidCorrection = drivePID.calculate(wheelMetersPerSecond, desiredState.speedMetersPerSecond);
            double outputVolts = MathUtil.clamp(feedforward + pidCorrection, -12, 12);

            io.setDriveVoltage(outputVolts);
        }
        else {
            io.setDriveVoltage(driveFeedforward.calculate(desiredState.speedMetersPerSecond));
        }

        double targetWheelAngleDegrees = desiredState.angle.getDegrees();
        double currentEncoderAngleDegrees = inputs.angleAbsolutePositionDegrees;

        io.setAngleVoltage(anglePID.calculate(currentEncoderAngleDegrees, targetWheelAngleDegrees));
    }


    public SwerveModuleState getState() {

        return new SwerveModuleState(
            inputs.driveVelocityMetersPerSecond,
            Rotation2d.fromDegrees(
                // Use the absolute encoder.
                inputs.angleAbsolutePositionDegrees)
            );
    }



    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            inputs.drivePositionMeters,
            // Use the absolute encoder.
            Rotation2d.fromDegrees(inputs.angleAbsolutePositionDegrees)
        );
        
    }

}
