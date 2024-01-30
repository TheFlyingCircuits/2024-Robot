package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import frc.robot.Constants.GyroConstants;

public class GyroIOPigeon implements GyroIO {
    private Pigeon2 pigeon;

    public GyroIOPigeon() {

        pigeon = new Pigeon2(GyroConstants.pigeonID, GyroConstants.pigeonCANbus);
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

        //TODO: we left off working on figuring out mountpose and other configs
        
        pigeon.getConfigurator().apply(configs);
        
    }

    @Override
    public void setRobotYaw(double degrees) {
        pigeon.setYaw(degrees);
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.robotYawDegrees = pigeon.getYaw().getValueAsDouble();
        inputs.robotPitchDegrees = pigeon.getPitch().getValueAsDouble();
        inputs.robotRollDegrees = pigeon.getRoll().getValueAsDouble();
        inputs.robotYawDegreesPerSecond = pigeon.getAngularVelocityZWorld().getValueAsDouble();
    }

}
