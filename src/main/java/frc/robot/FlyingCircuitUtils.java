package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Constants.VisionConstants;

public class FlyingCircuitUtils {

    public static Translation3d noteCameraCoordsFromRobotCoords(Translation3d robotCoords) {
        Transform3d robotAxesFromCamPerspective = VisionConstants.robotToNoteCamera.inverse();
        return robotCoords.rotateBy(robotAxesFromCamPerspective.getRotation()).plus(robotAxesFromCamPerspective.getTranslation());
    }

    public static Translation3d robotCoordsFromNoteCameraCoords(Translation3d noteCamCoords) {
        Transform3d camAxesFromRobotPerspective = VisionConstants.robotToNoteCamera;
        return noteCamCoords.rotateBy(camAxesFromRobotPerspective.getRotation()).plus(camAxesFromRobotPerspective.getTranslation());
    }

    /**
     * Generates a field relative pose for the closest pickup for auto-intaking a note
     * by drawing a straight line to the note.
     * Once the robot is at this position, the robot should be
     * able to track the note itself.
     * @param robot - Current translation of the robot.
     * @param note - Translation of the target note.
     * @param radiusMeters - Distance from the note of the output pose.
     */
    public static Pose2d pickupAtNote(Translation2d robot, Translation2d note, double radiusMeters) {
        //vector pointing from the note to the robot
        Translation2d noteToBot = robot.minus(note);

        Translation2d targetTranslation = note.interpolate(robot, radiusMeters/noteToBot.getNorm());

        return new Pose2d(targetTranslation, noteToBot.getAngle());
    }
}
