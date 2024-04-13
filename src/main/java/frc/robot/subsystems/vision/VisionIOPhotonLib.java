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

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.FlyingCircuitUtils;
import frc.robot.Constants.VisionConstants;

public class VisionIOPhotonLib implements VisionIO {
    

    List<PhotonCamera> tagCameras;
    List<PhotonPoseEstimator> poseEstimators;

    PhotonCamera noteCamera;

    public VisionIOPhotonLib() {
        noteCamera = new PhotonCamera("noteCamera");

        tagCameras = Arrays.asList(
            new PhotonCamera("shooterCamera"),
            new PhotonCamera("trapCamera"),
            new PhotonCamera("leftCamera"),
            new PhotonCamera("rightCamera")
        );

        poseEstimators = new ArrayList<PhotonPoseEstimator>();

        for (int i = 0; i < tagCameras.size(); i++) {
            poseEstimators.add(
                new PhotonPoseEstimator(
                    VisionConstants.aprilTagFieldLayout,
                    PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
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
     */
    private Matrix<N3, N1> getVisionStdDevs(double distToTargetMeters, boolean useMultitag) {

        double slopeStdDevMetersPerMeterX;
        double slopeStdDevMetersPerMeterY;


        if (DriverStation.isAutonomous()) {
            if (useMultitag) {
                slopeStdDevMetersPerMeterX = 0.06;
                slopeStdDevMetersPerMeterY = 0.06;
            }

            else {
                slopeStdDevMetersPerMeterX = 0.08;
                slopeStdDevMetersPerMeterY = 0.08;
            }
        }
        else {
            if (useMultitag) {
                slopeStdDevMetersPerMeterX = 0.008;
                slopeStdDevMetersPerMeterY = 0.008;
            }
            else {
                slopeStdDevMetersPerMeterX = 0.008;
                slopeStdDevMetersPerMeterY = 0.008;
            }
        }

        double slopeStdDevRadiansPerMeter = 1000;

        //corresponds to x, y, and rotation standard deviations (meters and radians)
        return VecBuilder.fill(
            slopeStdDevMetersPerMeterX*distToTargetMeters,
            slopeStdDevMetersPerMeterY*distToTargetMeters,
            slopeStdDevRadiansPerMeter*distToTargetMeters
        );
    }


    /**
     * Generates a VisionMeasurement object based off of a camera and its pose estimator.
     * @param camera - PhotonCamera object of the camera you want a result from.
     * @param estimator - PhotonPoseEstimator that MUST correspond to the PhotonCamera.
     * @return - Optional VisionMeasurement. This is empty if the camera does not see a reliable target.
     */
    private Optional<VisionMeasurement> updateTagCamera(PhotonCamera camera, PhotonPoseEstimator estimator) {
        VisionMeasurement output = new VisionMeasurement();


        PhotonPipelineResult pipelineResult = camera.getLatestResult();
        if (!pipelineResult.hasTargets()) {
            return Optional.empty();
        }

        Optional<EstimatedRobotPose> poseEstimatorResult = estimator.update();
        if (poseEstimatorResult.isEmpty()) {
            return Optional.empty();
        }

        //either use multitag or
        //if there's only one tag on the screen, only trust it if its pose ambiguity is below threshold
        //photonPoseEstimator automatically switches techniques when detecting different number of tags
        if (pipelineResult.targets.size() == 1 && pipelineResult.getBestTarget().getPoseAmbiguity() > 0.2) {
            return Optional.empty();
        }

        output.nearestTagDistanceMeters = pipelineResult.getBestTarget().getBestCameraToTarget().getTranslation().getDistance(new Translation3d());
        
        if (output.nearestTagDistanceMeters > 7) {
            return Optional.empty();
        }


        output.robotFieldPose = poseEstimatorResult.get().estimatedPose.toPose2d();
        output.timestampSeconds = poseEstimatorResult.get().timestampSeconds;
        output.stdDevs = getVisionStdDevs(output.nearestTagDistanceMeters, (pipelineResult.targets.size() > 1));  //different standard devs for different methods of detecting apriltags
        output.cameraName = camera.getName();

        return Optional.of(output);
    }

    private Optional<Translation3d> updateIntakeCamera() {
        PhotonPipelineResult noteCameraResult = noteCamera.getLatestResult();
        if (!noteCameraResult.hasTargets()) {
            return Optional.empty();
        }

        // Negate the pitch and yaw that's reported by photon vision because
        // their convention isn't consistent with a right handed coordinate system.
        double nearestNoteYawDegrees = -noteCameraResult.getBestTarget().getYaw();
        double notePitchDegrees = -noteCameraResult.getBestTarget().getPitch();

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
            
        return Optional.of(note_robotFrame);
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
        Collections.sort(inputs.visionMeasurements, new Comparator<VisionMeasurement>() {
            @Override
            public int compare(VisionMeasurement o1, VisionMeasurement o2) {
                return -Double.compare(o1.stdDevs.get(0,0), o2.stdDevs.get(0,0));
            }
        });

        inputs.nearestNoteRobotFrame = updateIntakeCamera();
    }
}
