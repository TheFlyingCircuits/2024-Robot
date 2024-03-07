package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public interface VisionIO {

    @AutoLog
    public class VisionIOInputs {
        /**
         * Pose of the robot in meters and degrees, relative to the field. The origin is always centered on the right edge of the blue side, 
         * so that all coordinates on the field are positive.
         * <p>
         * When each axis is pointed at you, their corresponding angles are counterclockwise positive.
         * For example, when looking down at the z-axis, the yaw is counterclockwise positive.
         */
        public Pose2d robotFieldPose = new Pose2d();

        /**
         * Timestamp of when the robotFieldPose has been updated
         */
        public double timestampSeconds = 0.;

        /**
         * Distance from the camera to the nearest tag, in meters.
         */
        public double nearestTagDistanceMeters = 0;
        

        /**
         * Whether or not the intake camera currently sees a note to target.
         */
        public boolean intakeSeesNote = false;
        
        /**
         * Yaw of the center point of the nearest note RELATIVE TO THE CAMERA detected on the intake camera.
         * This value is positive to the left, and has maximum value of the camera's FOV/2.
         */
        public double nearestNoteYawDegrees = 0.;

    }

    public default void updateInputs(VisionIOInputs inputs) {};

}
