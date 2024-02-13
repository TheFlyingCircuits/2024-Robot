package frc.robot.subsystems.drivetrain;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

public interface GyroIO {

    @AutoLog
    public class GyroIOInputs {
        /** 
         * The yaw rotation of the robot, as a Rotation2d.
         * A counterclockwise rotation will result in a positive increase in this value.
         * An angle of 0 represents the robot facing away from the blue alliance wall. 
         * */
        public Rotation2d robotYawRotation2d = new Rotation2d(0.0);
        

        /**
         * The angular velocity of the robot's yaw, as a Rotation2d per second.
         * A counterclockwise rotation will result in a positive value.
         */
        public Rotation2d robotYawRotation2dPerSecond = new Rotation2d(0.0);

        

        //temporary values, may be used for climbing
        public Rotation2d robotPitchRotation2d = new Rotation2d(0.0);
        public Rotation2d robotRollRotation2d = new Rotation2d(0.0);
    }
    
    public default void updateInputs(GyroIOInputs inputs) {};

    /**
     * Sets the saved yaw position of the robot, in degrees.
     * Your robot will now read its current angular position as the value you pass into this function.
     * Typically, an angle of 0 should be facing away from the blue alliance wall.
     */
    public default void setRobotYaw(double degrees) {};

    //this shouldn't need to be implemented by anything besides the sim implementation
    public default void calculateYaw(SwerveModulePosition[] modulePositions) {};
}