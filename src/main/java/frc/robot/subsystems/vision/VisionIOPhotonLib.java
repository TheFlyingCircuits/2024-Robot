package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Constants.VisionConstants;

public class VisionIOPhotonLib implements VisionIO {
    
    PhotonCamera shooterCamera;
    PhotonCamera trapCamera;
    PhotonCamera noteCamera;
    PhotonPoseEstimator shooterPoseEstimator;
    PhotonPoseEstimator trapPoseEstimator;

    public VisionIOPhotonLib() {
        shooterCamera = new PhotonCamera("shooterCamera");
        trapCamera = new PhotonCamera("trapCamera");
        noteCamera = new PhotonCamera("noteCamera");

        shooterPoseEstimator = new PhotonPoseEstimator(
            VisionConstants.aprilTagFieldLayout, 
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            shooterCamera,
            VisionConstants.robotToShooterCamera
        );

        trapPoseEstimator = new PhotonPoseEstimator(
            VisionConstants.aprilTagFieldLayout, 
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            trapCamera,
            VisionConstants.robotToTrapCamera
        );
    }

    /**
     * Calculates a matrix of standard deviations of the vision pose estimate, in meters and degrees. 
     * This is a function of the distance from the camera to the april tag.
     * @param distToTargetMeters - Distance from the camera to the apriltag.
     */
    private Matrix<N3, N1> getVisionStdDevs(double distToTargetMeters, boolean useMultitag) {

        double slopeStdDevMetersPerMeterX;
        double slopeStdDevMetersPerMeterY;


        if (useMultitag) {
            slopeStdDevMetersPerMeterX = 0.001;
            slopeStdDevMetersPerMeterY = 0.003;
        }

        else {
            slopeStdDevMetersPerMeterX = 0.5;
            slopeStdDevMetersPerMeterY = 0.5;
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
    private Optional<VisionMeasurement> updateCamera(PhotonCamera camera, PhotonPoseEstimator estimator) {
        VisionMeasurement output = new VisionMeasurement();


        PhotonPipelineResult shooterCameraResult = camera.getLatestResult();
        if (!shooterCameraResult.hasTargets())
            return Optional.empty();

        Optional<EstimatedRobotPose> poseEstimatorResult = estimator.update();
        if (poseEstimatorResult.isEmpty())
            return Optional.empty();

        //either use multitag or
        //if there's only one tag on the screen, only trust it if its pose ambiguity is below threshold
        //photonPoseEstimator automatically switches techniques when detecting different number of tags
        if (shooterCameraResult.targets.size() == 1 && shooterCameraResult.getBestTarget().getPoseAmbiguity() > 0.2)
            return Optional.empty();

        output.nearestTagDistanceMeters = shooterCameraResult.getBestTarget().getBestCameraToTarget().getTranslation().getDistance(new Translation3d());
        
        if (output.nearestTagDistanceMeters > 4)
            return Optional.empty();


        output.robotFieldPose = poseEstimatorResult.get().estimatedPose.toPose2d();
        output.timestampSeconds = poseEstimatorResult.get().timestampSeconds;
        output.stdDevs = getVisionStdDevs(output.nearestTagDistanceMeters, (shooterCameraResult.targets.size() > 1));  //different standard devs for different methods of detecting apriltags

        return Optional.of(output);
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        inputs.visionMeasurements = new ArrayList<VisionMeasurement>();
        
        Optional<VisionMeasurement> shooterResult = updateCamera(shooterCamera, shooterPoseEstimator);
        if (shooterResult.isPresent())
            inputs.visionMeasurements.add(shooterResult.get());

        Optional<VisionMeasurement> trapResult = updateCamera(trapCamera, trapPoseEstimator);
        if (trapResult.isPresent())
            inputs.visionMeasurements.add(trapResult.get());


        //sorts visionMeasurements by standard deviations in the x direction
        Collections.sort(inputs.visionMeasurements, new Comparator<VisionMeasurement>() {
            @Override
            public int compare(VisionMeasurement o1, VisionMeasurement o2) {
                return Double.compare(o1.stdDevs.get(0,0), o2.stdDevs.get(0,0));
            }
        });


        PhotonPipelineResult noteCameraResult = noteCamera.getLatestResult();
        if (!noteCameraResult.hasTargets()) {
            inputs.intakeSeesNote = false;
            inputs.nearestNoteYawDegrees = 0;
        }
        else {
            inputs.intakeSeesNote = true;
            inputs.nearestNoteYawDegrees = -noteCameraResult.getBestTarget().getYaw();
        }
        
    };
}
