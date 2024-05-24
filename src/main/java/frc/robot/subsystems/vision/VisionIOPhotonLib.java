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

        poseEstimators = new ArrayList<PhotonPoseEstimator>();

        for (int i = 0; i < tagCameras.size(); i++) {
            poseEstimators.add(
                new PhotonPoseEstimator(
                    VisionConstants.aprilTagFieldLayout,
                    PoseStrategy.AVERAGE_BEST_TARGETS,//PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
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

        updateDemoModeTagLayout(estimator, seenTags, poseEstimate.estimatedPose);
        // updateDemoModeFieldOrigin(estimator, seenTags, poseEstimate.estimatedPose);
        Logger.recordOutput("demoMode/robotFullPose", poseEstimate.estimatedPose);

        return Optional.of(output);
    }


    private void updateDemoModeFieldOrigin(PhotonPoseEstimator estimator, List<PhotonTrackedTarget> seenTags, Pose3d robotPose_fieldFrame) {
        if (!Constants.isDemoMode) {
            return;
        }

        Transform3d tagAxes_cameraFrame = null;
        for (PhotonTrackedTarget tag : seenTags) {
            if (tag.getFiducialId() == FieldElement.demoTargetID) {
                tagAxes_cameraFrame = tag.getBestCameraToTarget();
                break;
            }
        }

        if (tagAxes_cameraFrame == null) {
            return;
        }

        Transform3d camAxes_robotFrame = estimator.getRobotToCameraTransform();
        Transform3d tagAxes_robotFrame = camAxes_robotFrame.plus(tagAxes_cameraFrame);
        Logger.recordOutput("demoMode/tagLocationRobotFrame", tagAxes_robotFrame.getTranslation());
        Transform3d robotAxes_fieldFrame = new Transform3d(robotPose_fieldFrame.getTranslation(), robotPose_fieldFrame.getRotation());

        Transform3d tagAxes_fieldFrame = robotAxes_fieldFrame.plus(tagAxes_robotFrame);

        Pose3d tagPose_fieldFrame = new Pose3d(tagAxes_fieldFrame.getTranslation(), tagAxes_fieldFrame.getRotation());

        Pose3d shouldBeTagPose_fieldFrame = VisionConstants.aprilTagFieldLayout.getTagPose(FieldElement.demoTargetID).get();

        Transform3d poseDiff = new Transform3d(tagPose_fieldFrame, shouldBeTagPose_fieldFrame);
        Pose3d modifiedOrigin = new Pose3d().transformBy(poseDiff);


        AprilTagFieldLayout sortaUpdatedLayout = new AprilTagFieldLayout(VisionConstants.aprilTagFieldLayout.getTags(), VisionConstants.aprilTagFieldLayout.getFieldLength(), VisionConstants.aprilTagFieldLayout.getFieldWidth());
        sortaUpdatedLayout.setOrigin(modifiedOrigin);
        estimator.setFieldTags(sortaUpdatedLayout);


        Pose3d thisOneWorks = estimator.getFieldTags().getTagPose(FieldElement.demoTargetID).get();
        // Pose3d thisOneWorksToo = estimator.getFieldTags().getTagPose(buddyTagID).get();


        ArrayList<Pose3d> posesToLog = new ArrayList<>();
        posesToLog.add(thisOneWorks);
        Logger.recordOutput("demoMode/demoTagsFieldPose", posesToLog.toArray(new Pose3d[0]));
        Logger.recordOutput("demoMode/origin", modifiedOrigin);
    }


    private void updateDemoModeTagLayout(PhotonPoseEstimator estimator, List<PhotonTrackedTarget> seenTags, Pose3d robotPose_fieldFrame) {
        if (!Constants.isDemoMode) {
            return;
        }

        boolean has4 = false;
        boolean has3 = false;
        Transform3d tagAxesInCameraFrame = null;
        for (PhotonTrackedTarget tag : seenTags) {
            if (tag.getFiducialId() == FieldElement.demoTargetID) {
                tagAxesInCameraFrame = tag.getBestCameraToTarget();
            }

            if (tag.getFiducialId() == 4) {
                has4 = true;
            }

            if (tag.getFiducialId() == 3) {
                has3 = true;
            }
        }

        if (tagAxesInCameraFrame == null) {
            return;
        }

        Transform3d camAxesInRobotFrame = estimator.getRobotToCameraTransform();
        Transform3d tagAxesInRobotFrame = camAxesInRobotFrame.plus(tagAxesInCameraFrame);
        Logger.recordOutput("demoMode/tagLocationRobotFrame", tagAxesInRobotFrame.getTranslation());

        Translation3d tagX_robotFrame = new Translation3d(1, 0, 0).rotateBy(tagAxesInRobotFrame.getRotation());
        Translation3d tagY_robotFrame = new Translation3d(0, 1, 0).rotateBy(tagAxesInRobotFrame.getRotation());
        Translation3d tagZ_robotFrame = new Translation3d(0, 0, 1).rotateBy(tagAxesInRobotFrame.getRotation());
        Logger.recordOutput("demoMode/tagX_robotFrame", tagX_robotFrame);
        Logger.recordOutput("demoMode/tagY_robotFrame", tagY_robotFrame);
        Logger.recordOutput("demoMode/tagZ_robotFrame", tagZ_robotFrame);

        double demoTagFieldX = FieldElement.SPEAKER.getX();
        double demoTagFieldY = FieldElement.SPEAKER.getY();
        double demoTagFieldZ = tagAxesInRobotFrame.getTranslation().getZ();
        Translation3d demoTagFieldLocation = new Translation3d(demoTagFieldX, demoTagFieldY, demoTagFieldZ);

        // don't have the robot think it's sinking into the ground if the tag isn't straight up and down.
        // we only care about the yaw being correct? I think this is thr right way, but i'm not 100% sure.
        // TODO: this shouldn't work becuase extrinsic rotations about robot frame axes won't be the same
        //       as extrinsic rotations about field frame axes? However, this still seems to work!
        double demoTagFieldYaw = FieldElement.SPEAKER.getOrientation().getZ();
        double demoTagFieldPitch = tagAxesInRobotFrame.getRotation().getY();
        double demoTagFieldRoll = tagAxesInRobotFrame.getRotation().getX();
        Rotation3d demoTagFieldOrientation = new Rotation3d(demoTagFieldRoll, demoTagFieldPitch, demoTagFieldYaw);
        // demoTagFieldOrientation = FieldElement.SPEAKER.getOrientation(); // <- I've at least confirmed that this is wrong!
        AprilTag demoTag = new AprilTag(FieldElement.demoTargetID, new Pose3d(demoTagFieldLocation, demoTagFieldOrientation));


        int buddyTagID = 3;
        double distanceBetweenDemoTags = Units.inchesToMeters(22.25);
        Translation3d demoTagYAxis_fieldFrame = new Translation3d(0, 1, 0).rotateBy(demoTagFieldOrientation);
        Translation3d demoTagToBuddyTag_fieldFrame = demoTagYAxis_fieldFrame.times(distanceBetweenDemoTags);
        AprilTag demoTagBuddy = new AprilTag(buddyTagID, new Pose3d(demoTagFieldLocation.plus(demoTagToBuddyTag_fieldFrame), demoTagFieldOrientation));

        ArrayList<Pose3d> posesToLog = new ArrayList<>();
        if (has4) {
            posesToLog.add(demoTag.pose);
        }
        if (has3) {
            posesToLog.add(demoTagBuddy.pose);
        }
        Logger.recordOutput("demoMode/demoTagsFieldPose", posesToLog.toArray(new Pose3d[0]));


        AprilTag dummyTag = new AprilTag(1, new Pose3d());
        AprilTagFieldLayout updatedLayout = new AprilTagFieldLayout(Arrays.asList(dummyTag, demoTag, demoTagBuddy), VisionConstants.aprilTagFieldLayout.getFieldLength(), VisionConstants.aprilTagFieldLayout.getFieldWidth());
        estimator.setFieldTags(updatedLayout);

        boolean endNow = true;

        if (endNow) {
            return;
        }

        // when using the setOrigin() strategy, I have to use the full field of tags as is.
        // they all rotate and translate together. This is becuase updates to the tag layout are still not honored.
        double normalHeight = VisionConstants.aprilTagFieldLayout.getTagPose(4).get().getZ();
        double measuredHeight = tagAxesInRobotFrame.getTranslation().getZ();
        double heightShift = normalHeight - measuredHeight; // if tags are lower to the ground (measuredHeight < normalHeight), then the origin tied to the tags is also pushed into the floor, and the robot rises relative to that origin. If I make a new origin that also rises off the ground with the robot by the same amount, then the robot hasn't risen relative to that new origin at all? This still feels a bit backwards to me, I'll think about it later.
        Translation3d originLocation = new Translation3d(0, 0, heightShift);


        double robotYaw = robotPose_fieldFrame.getZ();
        Rotation3d tagOrientaiton_oppositeFieldFrame = tagAxesInRobotFrame.getRotation().rotateBy(new Rotation3d(0, 0, -robotYaw));
        Rotation3d originOrientation = new Rotation3d(tagOrientaiton_oppositeFieldFrame.getX(), tagOrientaiton_oppositeFieldFrame.getY(), 0);//tagAxesInRobotFrame.getRotation();
        Pose3d originAxes = new Pose3d(originLocation, originOrientation);

        AprilTagFieldLayout sortaUpdatedLayout = new AprilTagFieldLayout(VisionConstants.aprilTagFieldLayout.getTags(), VisionConstants.aprilTagFieldLayout.getFieldLength(), VisionConstants.aprilTagFieldLayout.getFieldWidth());
        sortaUpdatedLayout.setOrigin(originAxes);
        estimator.setFieldTags(sortaUpdatedLayout);

        Pose3d thisOneWorks = estimator.getFieldTags().getTagPose(demoTag.ID).get();
        Pose3d thisOneWorksToo = estimator.getFieldTags().getTagPose(buddyTagID).get();


        // ArrayList<Pose3d> posesToLog = new ArrayList<>();
        if (has4) {
            posesToLog.add(thisOneWorks);
        }
        if (has3) {
            posesToLog.add(thisOneWorksToo);
        }
        Logger.recordOutput("demoMode/demoTagsFieldPose", posesToLog.toArray(new Pose3d[0]));
        Logger.recordOutput("demoMode/origin", originAxes);





        // double demoOffsetInches = SmartDashboard.getNumber("demoTargetOffset", 12);
        // SmartDashboard.putNumber("demoTargetOffset", demoOffsetInches);
        double verticalOffsetMeters = Units.inchesToMeters(8);
        FieldElement.demoTargetLocation = demoTagFieldLocation.plus(new Translation3d(0, 0, verticalOffsetMeters));
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
