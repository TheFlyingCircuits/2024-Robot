// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.SwerveModuleConstants;

/** Add your docs here. */
public class SwerveModuleIOSim implements SwerveModuleIO {
    //TODO: fiddle around with rotational inertias for more realism (try increasing drive rotational inertia)
    private FlywheelSim angleSim = new FlywheelSim(DCMotor.getNEO(1), 1./SwerveModuleConstants.steerGearReduction, 0.025);
    private FlywheelSim driveSim = new FlywheelSim(DCMotor.getNEO(1), 1./SwerveModuleConstants.driveGearReduction, 0.004);

    @Override
    public void updateInputs(SwerveModuleIOInputs inputs) {
        angleSim.update(0.02);
        driveSim.update(0.02); //TODO: add 0.02 to constants as loop time

        //DRIVE MOTOR ----
        //converts rpm of wheel into m/s of wheel
        double driveVelocityConversionFactor =  
            1./60. * SwerveModuleConstants.wheelCircumferenceMeters;
        
        inputs.driveVelocityMetersPerSecond = 
            driveSim.getAngularVelocityRPM()*driveVelocityConversionFactor;

        //calculate wheel position from (wheel velocity)*dt
        inputs.drivePositionMeters +=
            inputs.driveVelocityMetersPerSecond*0.02;

        
        //ANGLE MOTOR ----

        //converts rpm of module into deg/s of swerve module
        double angleVelocityConversionFactor =
            360.0/60.0;

        double angleVelocityDegreesPerSecond = 
            angleSim.getAngularVelocityRPM()*angleVelocityConversionFactor;

        //calculate module angle from (angular velocity)*dt, then mod to keep within -180 and 180
        inputs.angleAbsolutePositionDegrees += angleVelocityDegreesPerSecond*0.02;
        inputs.angleAbsolutePositionDegrees = 
            MathUtil.inputModulus(inputs.angleAbsolutePositionDegrees, -180, 180);

    }

    @Override
    public void setDriveVoltage(double volts) {
        driveSim.setInputVoltage(volts);
    }

    @Override
    public void setAngleVoltage(double volts) {
        angleSim.setInputVoltage(volts);
    }
}
