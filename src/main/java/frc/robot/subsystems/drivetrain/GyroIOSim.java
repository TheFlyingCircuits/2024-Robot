// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.Constants.DrivetrainConstants;

/** Add your docs here. */
public class GyroIOSim implements GyroIO {

    private double yawDegrees = 0.0;
    private double adjustmentDegrees = 0.0;
    private SwerveModulePosition[] prevModulePositions;

    /**
     * Because the gyroscope does not exist in simulation, this implementation serves to simulate the gyro.
     * <p>
     * Using odometry, the yaw can be calculated. In order to do this, call {@code calculateYawFromOdometry()} periodically.
     */
    public GyroIOSim(SwerveModulePosition[] modulePosition) {
        prevModulePositions = modulePosition;
    };

    /**
     * Because the gyroscope does not exist in simulation, this implementation serves to simulate the gyro.
     * <p>
     * Using odometry, the yaw can be calculated. In order to do this, call {@code calculateYawFromOdometry()} periodically.
     */
    public GyroIOSim() {
        prevModulePositions = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            prevModulePositions[i] = 
                new SwerveModulePosition(
                    0,
                    new Rotation2d(0)
                );
        }
    }

    /**
     * Updates yaw of simulated gyro using odometry. Call this method periodically in order for it to work.
     * @param modulePositions - an array of current module positions, in the same order as your SwerveDriveKinematics object
     */
    public void calculateYaw(SwerveModulePosition[] modulePositions) {
        SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[modulePositions.length];

        for (int i = 0; i < modulePositions.length; i++) {
            moduleDeltas[i] = 
                new SwerveModulePosition(
                    modulePositions[i].distanceMeters - prevModulePositions[i].distanceMeters,
                    modulePositions[i].angle); //the angle here should NOT be the change in angle, draw it out if you can't see why

        }

        Twist2d twist = DrivetrainConstants.swerveKinematics.toTwist2d(moduleDeltas);

        yawDegrees += Math.toDegrees(twist.dtheta);

        prevModulePositions = modulePositions;
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.robotYawRotation2d = Rotation2d.fromDegrees(yawDegrees + adjustmentDegrees);
    };

    

    /** Adds angle to yawDegrees..
     * @param angleDegrees - the angle to add to yawDegrees.
     */
    @Override
    public void setRobotYaw(double angleDegrees) {
        adjustmentDegrees = angleDegrees;
    };
}
