package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import frc.robot.Constants.GyroConstants;;

public class GyroIOPigeon {
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

        //TODO: we left off working on figuring out mountpose and other configs
        
        pigeon.getConfigurator().apply(configs);
        
    }
}
