package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.FlyingCircuitUtils;
import frc.robot.Constants.FieldElement;
import frc.robot.Constants.VisionConstants;

public class VisionIOPhotonLib implements VisionIO {
    

    List<PhotonCamera> tagCameras;
    List<PhotonPoseEstimator> poseEstimators;

    PhotonCamera noteCamera;

    public VisionIOPhotonLib() {
        noteCamera = new PhotonCamera("noteCamera");

        tagCameras = Arrays.asList(
            new PhotonCamera(VisionConstants.cameraNames[0]),
            new PhotonCamera(VisionConstants.cameraNames[1])
            // new PhotonCamera(VisionConstants.cameraNames[2]),
            // new PhotonCamera(VisionConstants.cameraNames[3])
        );

        /* When in demo mode, the apriltags will probably be pitched/rolled a bit
         * relative to their normal vertical orientation because they will be held
         * by a person running the demo rather than being mounted to a wall.
         * The tags may also be at a different height than normal.
         * 
         * In order to still measure the robot's "field oreinted pose" accurately,
         * we must inform the pose estimators of the new pitch/roll/height of the tags
         * by updating the TagLayout. However, I've discovered through testing that
         * updated tag layouts involving more than one tag are only taken into account
         * when running pose estimation on the rio itself, and aren't taken into account
         * when running pose estimation on a co-processor. To get around this, we use
         * an alternative pose estimation strategy when in demo mode.
         */
        PoseStrategy estimationStrat = PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;
        if (Constants.isDemoMode) {
            // TODO: learn about "averaging" 3D orientations?
            //       it seems like it's not super straight forward,
            //       but I don't have time for a rabbit hole right now.
            estimationStrat = PoseStrategy.AVERAGE_BEST_TARGETS;
        }

        poseEstimators = new ArrayList<PhotonPoseEstimator>();
        for (int i = 0; i < tagCameras.size(); i++) {
            poseEstimators.add(
                new PhotonPoseEstimator(
                    VisionConstants.aprilTagFieldLayout,
                    estimationStrat,
                    tagCameras.get(i),
                    VisionConstants.tagCameraTransforms[i]
                )
            );
        }

    }

    /**
     * Calculates a matrix of standard deviations of the vision pose estimate, in meters and degrees. 
     * This is a function of the distance from the camera to the april tag.
     * @param distToTargetMeters - Distance from the camera to the apriltag.
     * @return A vector of the standard deviations given distance in X (m), Y (m), and Rotation (Rad)
     */
    private Matrix<N3, N1> getVisionStdDevs(double distToTargetMeters, boolean useMultitag) {

        double slopeStdDevMetersPerMeterX;
        double slopeStdDevMetersPerMeterY;



        // previous working vision
        
        if (useMultitag) {
            slopeStdDevMetersPerMeterX = 0.004;
            slopeStdDevMetersPerMeterY = 0.009;
        }
        else {
            slopeStdDevMetersPerMeterX = 0.008;
            slopeStdDevMetersPerMeterY = 0.008;
        }



        //bad vision from practice match at worlds

        // if (DriverStation.isAutonomous()) {
        //     if (useMultitag) {
        //         slopeStdDevMetersPerMeterX = 0.06;
        //         slopeStdDevMetersPerMeterY = 0.06;
        //     }

        //     else {
        //         slopeStdDevMetersPerMeterX = 0.08;
        //         slopeStdDevMetersPerMeterY = 0.08;
        //     }
        // }
        // else {
        //     if (useMultitag) {
        //         slopeStdDevMetersPerMeterX = 0.008;
        //         slopeStdDevMetersPerMeterY = 0.008;
        //     }
        //     else {
        //         slopeStdDevMetersPerMeterX = 0.008;
        //         slopeStdDevMetersPerMeterY = 0.008;
        //     }
        // }

        // previous linear model
        return VecBuilder.fill(
            slopeStdDevMetersPerMeterX*distToTargetMeters,
            slopeStdDevMetersPerMeterY*distToTargetMeters,
            99999
        );


        
        // double squareFactor = 0.5;
        // return VecBuilder.fill(0.04, 0.04, 99999);

        // // square model
        // return VecBuilder.fill(
        //     squareFactor*slopeStdDevMetersPerMeterX*Math.pow(distToTargetMeters, 3),
        //     squareFactor*slopeStdDevMetersPerMeterY*Math.pow(distToTargetMeters, 3),
        //     99999
        // );
    }


    /**
     * Generates a VisionMeasurement object based off of a camera and its pose estimator.
     * @param camera - PhotonCamera object of the camera you want a result from.
     * @param estimator - PhotonPoseEstimator that MUST correspond to the PhotonCamera.
     * @return - Optional VisionMeasurement. This is empty if the camera does not see a reliable target.
     */
    private Optional<VisionMeasurement> updateTagCamera(PhotonCamera camera, PhotonPoseEstimator estimator) {
        VisionMeasurement output = new VisionMeasurement();

        Optional<EstimatedRobotPose> poseEstimatorResult = estimator.update();
        if (poseEstimatorResult.isEmpty()) {
            return Optional.empty();
        }
        EstimatedRobotPose poseEstimate = poseEstimatorResult.get();
        List<PhotonTrackedTarget> seenTags = poseEstimate.targetsUsed;
        

        //either use multitag or
        //if there's only one tag on the screen, only trust it if its pose ambiguity is below threshold
        //photonPoseEstimator automatically switches techniques when detecting different number of tags
        if (seenTags.size() == 1 && seenTags.get(0).getPoseAmbiguity() > 0.2) {
            return Optional.empty();
        }
        
        for (PhotonTrackedTarget tag : seenTags) {
            double distance = tag.getBestCameraToTarget().getTranslation().getDistance(new Translation3d());
            output.averageTagDistanceMeters += distance/seenTags.size();
        }

        // don't add vision measurements that are too far away
        // for reference: it is 6 meters from speaker tags to wing.
        if (output.averageTagDistanceMeters > 7) {
            return Optional.empty();
        }


        output.robotFieldPose = poseEstimate.estimatedPose.toPose2d();
        output.timestampSeconds = poseEstimate.timestampSeconds;
        output.stdDevs = getVisionStdDevs(output.averageTagDistanceMeters, (seenTags.size() > 1));  //different standard devs for different methods of detecting apriltags
        output.cameraName = camera.getName();
        output.tagsUsed = new int[seenTags.size()];
        for (int i = 0; i < seenTags.size(); i += 1) {
            output.tagsUsed[i] = seenTags.get(i).getFiducialId();
        }

        updateDemoModeTagLayout(estimator, seenTags);
        Logger.recordOutput("demoMode/robotFullPose", poseEstimate.estimatedPose);

        return Optional.of(output);
    }


    private void updateDemoModeTagLayout(PhotonPoseEstimator estimator, List<PhotonTrackedTarget> seenTags) {
        // Do nothing if it isn't demo mode
        if (!Constants.isDemoMode) {
            return;
        }

        // Find the pose of the demo target in the robot's frame
        Transform3d tagPose_cameraFrame = null;
        for (PhotonTrackedTarget tag : seenTags) {
            if (tag.getFiducialId() == FieldElement.getSpeakerTagID()) {
                tagPose_cameraFrame = tag.getBestCameraToTarget();
            }
        }

        if (tagPose_cameraFrame == null) {
            return;
        }

        Transform3d camPose_robotFrame = estimator.getRobotToCameraTransform();
        Transform3d tagPose_robotFrame = camPose_robotFrame.plus(tagPose_cameraFrame);

        // Step 1) Determine the location of the demo tag on the field.
        //         We put it at the same XY locaiton as our speaker target,
        //         but use the height measured by the robot rather than the regulation height. 
        Translation3d tagLocation_fieldFrame = new Translation3d(FieldElement.SPEAKER.getX(), FieldElement.SPEAKER.getY(), tagPose_robotFrame.getZ());

        // We don't want the robot to think it's sinking into the ground if the person holding the
        // demo tag isn't holding it straight up and down, so we have to update the FieldLayout to be
        // aware of the tag's pitch and roll.
        //
        // Step 2) Determine the orientation of the demo tag relative to the field axes.
        //         We do this by finding how the field frame must be oriented w.r.t.
        //         the robot frame, and then transforming the orientation of the tag
        //         (as seen by the robot) into the field frame.
        //         Sorry that's kinda hard to understand, I can't think of better wording right now.
        //         
        //         robotYaw_wrtFieldFrame = tagYaw_wrtFieldFrame + "angle from tagX to robotX as measured in the plane of the floor" (this phrasing accounts for the demo tag's z axis not pointing straight up from the floor to the sky, because robotYaw_wrtTagFrame doesn't measure in the plane of the floor when the tag frame's z axis isn't coincident with the floor normal).
        //         robotYaw_wrtFieldFrame = tagYaw_wrtFieldFrame - tagYaw_wrtRobotFrame
        //         fieldYaw_wrtRobotFrame = tagYaw_wrtRobotFrame - tagYaw_wrtFieldFrame
        Rotation2d tagYaw_robotFrame = tagPose_robotFrame.getRotation().toRotation2d();
        Rotation2d tagYaw_fieldFrame = FieldElement.SPEAKER.getOrientation().toRotation2d();
        Rotation2d fieldYaw_robotFrame = tagYaw_robotFrame.minus(tagYaw_fieldFrame);


        // To take a direciton vector from the robot frame and transform it into the field frame,
        // we find the rotation matrix whose columns are the robot's axes as measured in the field frame,
        // and then pump the direction vectors as seen in the robot frame through that matrix.
        // Sorry again that this is so wordy, this is really best described with pictures!
        // In other words, we have tagOrientation in terms of <robot_xDirection, robot_yDirection, robot_zDirection>.
        // In order to get tagOrientation in terms of <field_xDirection, field_yDirection, field_zDirection>
        // we simply write the robot's direction vectors in terms of the field's direction vectors, then collect like terms / simplify!
        Rotation3d fieldOrientation_robotFrame = new Rotation3d(0, 0, fieldYaw_robotFrame.getRadians());
        Rotation3d tagOrientation_robotFrame = tagPose_robotFrame.getRotation();
        Rotation3d tagOrientation_fieldFrame = tagOrientation_robotFrame.rotateBy(fieldOrientation_robotFrame.unaryMinus());

        // update the field layout and demo target location
        AprilTag demoTag = new AprilTag(FieldElement.getSpeakerTagID(), new Pose3d(tagLocation_fieldFrame, tagOrientation_fieldFrame));
        AprilTagFieldLayout updatedLayout = new AprilTagFieldLayout(Arrays.asList(demoTag), VisionConstants.aprilTagFieldLayout.getFieldLength(), VisionConstants.aprilTagFieldLayout.getFieldWidth());
        estimator.setFieldTags(updatedLayout);

        // aim 8 inches above the tag so we can hit an apple of of someone's head for the demo!
        Translation3d demoTargetOffset = new Translation3d(0, 0, Units.inchesToMeters(24));
        FieldElement.demoTargetLocation = tagLocation_fieldFrame.plus(demoTargetOffset);

        // record some debug info
        ArrayList<Pose3d> posesToLog = new ArrayList<>();
        posesToLog.add(demoTag.pose);


        if (demoTag.ID == 4) {
            // add the second speaker tag if we're using two for the demo (when using only using 1 demo tag, it has id 13)
            // it's offset from the first speaker tag, but should have the same orientaiton.
            double distanceBetweenSpeakerTags = Units.inchesToMeters(22.25);
            Translation3d tagYAxis_fieldFrame = new Translation3d(0, 1, 0).rotateBy(tagOrientation_fieldFrame);
            Translation3d secondTagLocation_fieldFrame = tagLocation_fieldFrame.plus(tagYAxis_fieldFrame.times(distanceBetweenSpeakerTags));
            AprilTag secondDemoTag = new AprilTag(3, new Pose3d(secondTagLocation_fieldFrame, tagOrientation_fieldFrame));

            updatedLayout = new AprilTagFieldLayout(Arrays.asList(demoTag, secondDemoTag), VisionConstants.aprilTagFieldLayout.getFieldLength(), VisionConstants.aprilTagFieldLayout.getFieldWidth());
            estimator.setFieldTags(updatedLayout);

            // Aim above the midpoint of both tags.
            FieldElement.demoTargetLocation = tagLocation_fieldFrame.plus(secondTagLocation_fieldFrame).div(2).plus(demoTargetOffset);

            for (PhotonTrackedTarget tag : seenTags) {
                if (tag.getFiducialId() == 3) {
                    posesToLog.add(secondDemoTag.pose);
                    break;
                }
            }

            // Note: you can get bad updates that make the robot think it's sinking into the floor
            //       if you rotate the tag in a way that it can't see 4, but it can see 3.
            //       In this scenario, the field layout doesn't get updated because 4 can't be seen,
            //       but the robot's pose is still updated based on tag 3, which it can see.
            //       It will be based off the last updated layout, which could cause it to sink
            //       into the floor. Not sure if I want to solve this problem.
            //       We're good as long as he can see both tags, or just number 4.
            //       We only get a problem if he can't see tag 3.
        }

        // logging
        Logger.recordOutput("demoMode/demoTagsFieldPose", posesToLog.toArray(new Pose3d[0]));
        Logger.recordOutput("demoMode/tagLocation_robotFrame", tagPose_robotFrame.getTranslation());

        Translation3d tagX_robotFrame = new Translation3d(1, 0, 0).rotateBy(tagPose_robotFrame.getRotation());
        Translation3d tagY_robotFrame = new Translation3d(0, 1, 0).rotateBy(tagPose_robotFrame.getRotation());
        Translation3d tagZ_robotFrame = new Translation3d(0, 0, 1).rotateBy(tagPose_robotFrame.getRotation());
        Logger.recordOutput("demoMode/tagX_robotFrame", tagX_robotFrame);
        Logger.recordOutput("demoMode/tagY_robotFrame", tagY_robotFrame);
        Logger.recordOutput("demoMode/tagZ_robotFrame", tagZ_robotFrame);
        Logger.recordOutput("demoMode/tagX_robotFrameMag", tagX_robotFrame.getNorm());
        Logger.recordOutput("demoMode/tagY_robotFrameMag", tagY_robotFrame.getNorm());
        Logger.recordOutput("demoMode/tagZ_robotFrameMag", tagZ_robotFrame.getNorm());
    }

    private List<Translation3d> updateIntakeCamera() {

        List<Translation3d> detectedNotes = new ArrayList<Translation3d>();

        PhotonPipelineResult noteCameraResult = noteCamera.getLatestResult();

        for (PhotonTrackedTarget target : noteCameraResult.getTargets()) {
            // Negate the pitch and yaw that's reported by photon vision because
            // their convention isn't consistent with a right handed coordinate system.
            double nearestNoteYawDegrees = -target.getYaw();
            double notePitchDegrees = -target.getPitch();

            // Use the reported pitch and yaw to calculate a unit vector in the camera
            // frame that points towards the note.
            Rotation3d directionOfNote = new Rotation3d(0, Math.toRadians(notePitchDegrees), Math.toRadians(nearestNoteYawDegrees));
            Translation3d unitTowardsNote = new Translation3d(1, directionOfNote);

            // Start the process of finding the full 3D distance from the camera to the note
            // by finding the coordinates of the normal vector of the floor,
            // as seen in the camera frame.
            Translation3d robotOrigin_robotFrame = new Translation3d();
            Translation3d aboveTheFloor_robotFrame = new Translation3d(0, 0, 1);
            Translation3d robotOrigin_camFrame = FlyingCircuitUtils.noteCameraCoordsFromRobotCoords(robotOrigin_robotFrame);
            Translation3d aboveTheFloor_camFrame = FlyingCircuitUtils.noteCameraCoordsFromRobotCoords(aboveTheFloor_robotFrame);
            Translation3d floorNormal_camFrame = aboveTheFloor_camFrame.minus(robotOrigin_camFrame);

            // Find where the vector that points from the camera to the note intersects
            // the plane of the floor.
            Translation3d floorAnchor = robotOrigin_camFrame;
            double distanceToNote = floorAnchor.toVector().dot(floorNormal_camFrame.toVector())
                                    / unitTowardsNote.toVector().dot(floorNormal_camFrame.toVector());

            // extend the original unit vector to the intersection point in the plane
            Translation3d note_camFrame = unitTowardsNote.times(distanceToNote);
            Translation3d note_robotFrame = FlyingCircuitUtils.robotCoordsFromNoteCameraCoords(note_camFrame);

            detectedNotes.add(note_robotFrame);
            // TODO: add desmos link?
        }


            
        return detectedNotes;
    }


    @Override
    public void updateInputs(VisionIOInputs inputs) {
        inputs.visionMeasurements = new ArrayList<VisionMeasurement>();
        
        for (int i = 0; i < tagCameras.size(); i++) {
            Optional<VisionMeasurement> camResult = updateTagCamera(
                tagCameras.get(i), poseEstimators.get(i)
            );

            if (camResult.isPresent()) {
                inputs.visionMeasurements.add(camResult.get());
            }
        }

        //sorts visionMeasurements by standard deviations in the x direction, biggest to smallest
        // Collections.sort(inputs.visionMeasurements, new Comparator<VisionMeasurement>() {
        //     @Override
        //     public int compare(VisionMeasurement o1, VisionMeasurement o2) {
        //         return -Double.compare(o1.stdDevs.get(0,0), o2.stdDevs.get(0,0));
        //     }
        // });

        inputs.visionMeasurements.sort(new Comparator<VisionMeasurement>() {
            public int compare(VisionMeasurement a, VisionMeasurement b) {
                return Double.compare(a.timestampSeconds, b.timestampSeconds);
            }
        });

        inputs.detectedNotesRobotFrame = updateIntakeCamera();
    }
}
