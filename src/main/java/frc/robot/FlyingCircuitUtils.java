package frc.robot;

import java.util.Arrays;
import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.FieldElement;
import frc.robot.Constants.VisionConstants;

/** idk where else to put the stuff in this class. I'll come back to it later. */
public class FlyingCircuitUtils {

    public static Pose2d getLocationOfFieldElement(FieldElement element) {

        Optional<Alliance> alliance = DriverStation.getAlliance();
        AprilTagFieldLayout fieldLayout = VisionConstants.aprilTagFieldLayout;

        if (alliance.isPresent() && alliance.get() == Alliance.Blue) {
            if (element == FieldElement.SPEAKER) { 
                // Target the opening of the speaker, rather than the speaker tag.
                double speakerDepthIntoField = Units.inchesToMeters(18.11);
                double x = (speakerDepthIntoField / 2);
                double y = Units.inchesToMeters(218.42);
                return new Pose2d(x, y, Rotation2d.fromDegrees(0));
             }
            if (element == FieldElement.AMP) { return fieldLayout.getTagPose(6).get().toPose2d(); }
            if (element == FieldElement.STAGE_LEFT) { return fieldLayout.getTagPose(15).get().toPose2d(); }
            if (element == FieldElement.STAGE_RIGHT) { return fieldLayout.getTagPose(16).get().toPose2d(); }
            if (element == FieldElement.CENTER_STAGE) { return fieldLayout.getTagPose(14).get().toPose2d(); }
            if (element == FieldElement.LOB_TARGET) {
                Pose2d speaker = getLocationOfFieldElement(FieldElement.SPEAKER);
                Pose2d amp = getLocationOfFieldElement(FieldElement.AMP);
                return speaker.interpolate(amp, 0.6);
            }
        }

        if (alliance.isPresent() && alliance.get() == Alliance.Red) {
            if (element == FieldElement.SPEAKER) {
                // Target the opening of the speaker, rather than the speaker tag.
                double speakerDepthIntoField = Units.inchesToMeters(18.11);
                double redAllianceWallX = Units.inchesToMeters(652.73-1.5);
                double x = redAllianceWallX - (speakerDepthIntoField / 2);
                double y = Units.inchesToMeters(218.42);
                return new Pose2d(x, y, Rotation2d.fromDegrees(180));
            }
            if (element == FieldElement.AMP) { return fieldLayout.getTagPose(5).get().toPose2d(); }
            if (element == FieldElement.STAGE_LEFT) { return fieldLayout.getTagPose(11).get().toPose2d(); }
            if (element == FieldElement.STAGE_RIGHT) { return fieldLayout.getTagPose(12).get().toPose2d(); }
            if (element == FieldElement.CENTER_STAGE) { return fieldLayout.getTagPose(13).get().toPose2d(); }
            if (element == FieldElement.LOB_TARGET) {
                Pose2d speaker = getLocationOfFieldElement(FieldElement.SPEAKER);
                Pose2d amp = getLocationOfFieldElement(FieldElement.AMP);
                return speaker.interpolate(amp, 0.6);
            }
        }

        // Should never get to this point as long as we're connected to the driver station.
        return new Pose2d();
    }

    public static Translation2d getVectorToFieldElement(FieldElement element, Pose2d yourPoseOnTheField) {
        if (element == FieldElement.CARPET) {
            // delta between you and right in front of you
            // is just a unit vector that points in the direction you're facing.
            return new Translation2d(yourPoseOnTheField.getRotation().getCos(), yourPoseOnTheField.getRotation().getSin());
        }
        Translation2d elementLocaiton = getLocationOfFieldElement(element).getTranslation();
        Translation2d yourLocation = yourPoseOnTheField.getTranslation();
        return elementLocaiton.minus(yourLocation);
    }

    /**
     * Draws a vector from the given pose to the desired field element,
     * then returns the angle of that vector as measured in the field coordinate system.
     * (i.e. angle is measured relative to the x axis of the field, which points away from the blue alliance wall).
     * @param element
     * @param vectorTail
     * @return
     */
    public static Rotation2d getAngleToFieldElement(FieldElement element, Pose2d yourPoseOnTheField) {
        return getVectorToFieldElement(element, yourPoseOnTheField).getAngle();
    }

    public static double getDistanceToFieldElement(FieldElement element, Pose2d yourPoseOnTheField) {
        return getVectorToFieldElement(element, yourPoseOnTheField).getNorm();
    }

    public static Pose2d getClosestTrap(Pose2d yourPoseOnTheField) {
        Pose2d[] trapLocations = {FlyingCircuitUtils.getLocationOfFieldElement(FieldElement.STAGE_LEFT),
                                  FlyingCircuitUtils.getLocationOfFieldElement(FieldElement.STAGE_RIGHT),
                                  FlyingCircuitUtils.getLocationOfFieldElement(FieldElement.CENTER_STAGE)};

        return yourPoseOnTheField.nearest(Arrays.asList(trapLocations));
    }

    public static Translation3d noteCameraCoordsFromRobotCoords(Translation3d robotCoords) {
        Transform3d robotAxesFromCamPerspective = VisionConstants.robotToNoteCamera.inverse();
        return robotCoords.rotateBy(robotAxesFromCamPerspective.getRotation()).plus(robotAxesFromCamPerspective.getTranslation());
    }

    public static Translation3d robotCoordsFromNoteCameraCoords(Translation3d noteCamCoords) {
        Transform3d camAxesFromRobotPerspective = VisionConstants.robotToNoteCamera;
        return noteCamCoords.rotateBy(camAxesFromRobotPerspective.getRotation()).plus(camAxesFromRobotPerspective.getTranslation());
    }
}
