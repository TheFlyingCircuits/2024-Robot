package frc.robot.subsystems.vision;

import java.util.ArrayList;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public interface VisionIO {

    public final class VisionMeasurement {



        /**
         * Pose of the robot in meters and degrees, relative to the field. The origin is always centered on the right edge of the blue side, 
         * so that all coordinates on the field are positive.
         * <p>
         * When each axis is pointed at you, their corresponding angles are counterclockwise positive.
         * For example, when looking down at the z-axis, the yaw is counterclockwise positive.
         */
        public Pose2d robotFieldPose;

        /**
         * FPGA timestamp of when the robotFieldPose has been updated
         */
        public double timestampSeconds;

        /**
         * Distance from the camera to the nearest tag, in meters.
         */
        public double nearestTagDistanceMeters;

        /**
         * Standard deviations of this vision measurement, in meters and radians.
         * Represents (X, Y, and rotation).
         */
        public Matrix<N3, N1> stdDevs;

        /**
         * Creates a new VisionMeasurement object. See the definition of this class for further documentation.
         */
        public VisionMeasurement(Pose2d robotFieldPose, double timestampSeconds, double nearestTagDistanceMeters, Matrix<N3, N1> stdDevs) {
            this.robotFieldPose=robotFieldPose;
            this.timestampSeconds=timestampSeconds;
            this.nearestTagDistanceMeters=nearestTagDistanceMeters;
            this.stdDevs=stdDevs;
        }

        /**
         * Creates a new VisionMeasurement object. See the definition of this class for further documentation.
         */
        public VisionMeasurement() {};


    }

    @AutoLog
    public class VisionIOInputs {

        /**
         * List of all vision measurements from the last frame. This is empty if no measurement is present.
         * This array is sorted by the standard deviation of the measurement.
         */
        public ArrayList<VisionMeasurement> visionMeasurements = new ArrayList<VisionMeasurement>();

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
