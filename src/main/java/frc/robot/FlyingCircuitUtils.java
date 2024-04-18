package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
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

    /**
     * Util method to create a path following command given the name of the path in pathplanner.
     * Make sure to call this after the AutoBuilder is configured.
     */
    public static Command followPath(String pathName) {
        return AutoBuilder.followPath(PathPlannerPath.fromPathFile(pathName));
    }

    /**
     * Returns true if the fed position is outside of the field.
     * @param toleranceMeters - Distance outside of the field that will still be considered "in the field"; i.e. the method will still return
     * true.
     */
    public static boolean isOutsideOfField(Translation2d pos, double toleranceMeters) {

        return (pos.getY() > 8.19 + toleranceMeters) || (pos.getY() < 0 - toleranceMeters)
            ||(pos.getX() > 16.54 + toleranceMeters) || (pos.getX() < 0 - toleranceMeters);
    }
}
