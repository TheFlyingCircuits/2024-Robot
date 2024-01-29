package frc.robot.subsystems.drivetrain;

import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {

    @AutoLog
    public class GyroIOInputs {
        /** 
         * The yaw rotation of the robot, in degrees.
         * A counterclockwise rotation will result in a positive increase in this value.
         * An angle of 0 represents the robot facing precisely away from your alliance wall. 
         * */
        public double robotYawDegrees = 0.0;
        

        /**
         * The angular velocity of the robot's yaw, in degrees per second.
         * A counterclockwise rotation will result in a positive value.
         */
        public double robotYawDegreesPerSecond = 0.0;

        

        //temporary values, may be used for climbing
        public double robotPitchDegrees = 0.0;
        public double robotRollDegrees = 0.0;
    }
    

    /**
     * Sets the saved yaw position of the robot, in degrees.
     * Your robot will now read its current angular position as the value you pass into this function.
     */
    public default void setRobotYaw(double degrees) {};
}