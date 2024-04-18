package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.GyroConstants;

public class GyroIOPigeon implements GyroIO {
    private Pigeon2 pigeon;

    private LinearFilter xAccelFilter;
    private LinearFilter yAccelFilter;
    private LinearFilter zAccelFilter;

    public GyroIOPigeon() {

        pigeon = new Pigeon2(GyroConstants.pigeonID, "CTRENetwork");
        configPigeon();

        xAccelFilter = LinearFilter.movingAverage(12);
        yAccelFilter = LinearFilter.movingAverage(12);
        zAccelFilter = LinearFilter.movingAverage(12);
    }


    private void configPigeon() {
        Pigeon2Configuration configs = new Pigeon2Configuration();

        /**
         * Setting the 'mountpose' or orientation the gyroscope is mounted on the robot
         */
        configs.MountPose.MountPoseYaw =  GyroConstants.mountPoseYawDegrees;
        configs.MountPose.MountPosePitch = GyroConstants.mountPosePitchDegrees;
        configs.MountPose.MountPoseRoll = GyroConstants.mountPoseRollDegrees;
        
        pigeon.getConfigurator().apply(configs);
        
    }

    @Override
    public void setRobotYaw(double degrees) {
        pigeon.setYaw(degrees);
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.robotYawRotation2d = Rotation2d.fromDegrees(pigeon.getYaw().getValueAsDouble());
        inputs.robotPitchRotation2d = Rotation2d.fromDegrees(pigeon.getPitch().getValueAsDouble());
        inputs.robotRollRotation2d = Rotation2d.fromDegrees(pigeon.getRoll().getValueAsDouble());
        inputs.robotYawRotation2dPerSecond = Rotation2d.fromDegrees(pigeon.getAngularVelocityZWorld().getValueAsDouble());
    

        //the pigeon's imu is affected by gravity, subtracting accounts for this.
        //the values from the pigeon are relative to the pigeon, so axes need to be switched around.
        inputs.robotAccelY = 9.81*(pigeon.getAccelerationX().getValueAsDouble() - pigeon.getGravityVectorX().getValueAsDouble());
        inputs.robotAccelY = yAccelFilter.calculate(inputs.robotAccelY);

        inputs.robotAccelZ = 9.81*(pigeon.getAccelerationY().getValueAsDouble() - pigeon.getGravityVectorY().getValueAsDouble());
        inputs.robotAccelZ = zAccelFilter.calculate(inputs.robotAccelZ);

        inputs.robotAccelX = 9.81*(pigeon.getAccelerationZ().getValueAsDouble() - pigeon.getGravityVectorZ().getValueAsDouble());
        inputs.robotAccelX = xAccelFilter.calculate(inputs.robotAccelX);
    }

}
