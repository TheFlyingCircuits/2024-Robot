package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.GyroConstants;

public class GyroIOPigeon implements GyroIO {
    private Pigeon2 pigeon;

    public GyroIOPigeon() {

        pigeon = new Pigeon2(GyroConstants.pigeonID, "CTRENetwork");
        configPigeon();
    }


    private void configPigeon() {
        Pigeon2Configuration configs = new Pigeon2Configuration();

        configs.Pigeon2Features.DisableNoMotionCalibration = false;
        configs.Pigeon2Features.DisableTemperatureCompensation = false;
        configs.Pigeon2Features.EnableCompass = false;

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
    }

}
