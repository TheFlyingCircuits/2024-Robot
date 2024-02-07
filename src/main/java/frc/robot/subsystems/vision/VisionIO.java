package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {

    @AutoLog
    public class VisionIOInputs {
        /**
         * Pose of the robot in meters and degrees, relative to the field. The origin is always centered on the right edge of the blue side, 
         * so that all coordinates on the field are positive.
         * <p>
         * This data comes as an array of {x, y, z, roll, pitch, yaw}.
         * <p>
         * When each axis is pointed at you, their corresponding angles are counterclockwise positive.
         * For example, when looking down at the z-axis, the yaw is counterclockwise positive.
         */
        public double[] robotFieldPose = {0, 0, 0, 0, 0, 0};

        /**
         * Distance from the camera to the nearest tag, in meters.
         */
        public double nearestTagDistanceMeters = 0;

    }

    public default void updateInputs(VisionIOInputs inputs) {};

}
