package frc.robot.subsystems.vision;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.util.struct.Struct;
import frc.robot.Constants.VisionConstants;

public interface VisionIO {

    public final class VisionMeasurement {

        public String cameraName;

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

    public class VisionIOInputs {

        /**
         * List of all vision measurements from the last frame. 
         * This array is sorted by the standard deviation of the measurement, from biggest to smallest.
         * (I think? this is the order that you should apply measurements to the pose estimator)
         * If no measurements are present, the array will be empty.
         */
        public List<VisionMeasurement> visionMeasurements = new ArrayList<VisionMeasurement>();

        /**
         * The 3D coordinates of every note detected by the intake camera in the robot coordinate frame.
         * Positive X is forward
         * Positive Y is left
         * Positive Z is up
         */
        public List<Translation3d> detectedNotesRobotFrame = new ArrayList<Translation3d>();

    }

    //AdvantageKit's AutoLog doesn't support logging array lists or custom objects,
    //so we wrote our own logging methods. There is a better way of doing this using
    //WPILib structs, but I'm not sure exactly how to go about doing it.
    public class VisionIOInputsLogged extends VisionIOInputs implements LoggableInputs {
        
        public void toLog(LogTable table) {

            for (VisionMeasurement meas : visionMeasurements) {
                
                String rootString = meas.cameraName+"VisionMeasurement";

                table.put(rootString+"/RobotFieldPose", meas.robotFieldPose);
                table.put(rootString+"/TimestampSeconds", meas.timestampSeconds);
                table.put(rootString+"/NearestTagDistanceMeters", meas.nearestTagDistanceMeters);
                table.put(rootString+"/StdDevX", meas.stdDevs.get(0, 0));
                table.put(rootString+"/StdDevY", meas.stdDevs.get(1, 0));
                table.put(rootString+"/StdDevRot", meas.stdDevs.get(2, 0));
                table.put(rootString+"/CameraName", meas.cameraName);
            }

            for (int i = 0; i < detectedNotesRobotFrame.size(); i++) {

                Translation3d note = detectedNotesRobotFrame.get(i);

                table.put("DetectedNoteRobotFrame" + Integer.toString(i), note);
            }
        }

        public void fromLog(LogTable table) {




            for (int i = 0;;i++) {
                String rootString = VisionConstants.cameraNames[i]+"VisionMeasurement";

                //hacky way to check if this vision measurement doesn't exist
                if (table.get(rootString+"/RobotFieldPose", 0) == 0)
                    break;
                
                VisionMeasurement meas = new VisionMeasurement();

                meas.robotFieldPose = table.get(rootString+"/RobotFieldPose", meas.robotFieldPose);
                meas.timestampSeconds = table.get(rootString+"/TimestampSeconds", meas.timestampSeconds);
                meas.nearestTagDistanceMeters = table.get(rootString+"/NearestTagDistanceMeters", meas.nearestTagDistanceMeters);
                double stdDevX = table.get(rootString+"/StdDevX", meas.stdDevs.get(0, 0));
                double stdDevY = table.get(rootString+"/StdDevY", meas.stdDevs.get(1, 0));
                double stdDevRot = table.get(rootString+"/StdDevRot", meas.stdDevs.get(2, 0));
                meas.stdDevs = VecBuilder.fill(stdDevX, stdDevY, stdDevRot);
                meas.cameraName = table.get(rootString+"/CameraName", "");

                visionMeasurements.add(meas);
            }

            for (int i = 0;; i++) {

                String entryName = "DetectedNoteRobotFrame" + Integer.toString(i);

                if (table.get(entryName, 0) == 0) {
                    break;
                }

                Translation3d note = table.get("DetectedNoteRobotFrame" + Integer.toString(i), new Translation3d());
                detectedNotesRobotFrame.add(note);

            }
            
        }
    }

    public default void updateInputs(VisionIOInputs inputs) {};

}
