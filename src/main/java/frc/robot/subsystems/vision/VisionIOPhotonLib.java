package frc.robot.subsystems.vision;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Constants.VisionConstants;

public class VisionIOPhotonLib implements VisionIO {
    
    PhotonCamera shooterCamera;
    PhotonCamera intakeCamera;
    PhotonPoseEstimator photonPoseEstimator;

    public VisionIOPhotonLib() {
        shooterCamera = new PhotonCamera("frontCamera");
        intakeCamera = new PhotonCamera("intakeCamera");

        photonPoseEstimator = new PhotonPoseEstimator(
            VisionConstants.aprilTagFieldLayout, 
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            shooterCamera,
            VisionConstants.robotToCamera
        );
    }


    private void updateShooterCamera(VisionIOInputs inputs) {
        PhotonPipelineResult shooterCameraResult = shooterCamera.getLatestResult();
        if (!shooterCameraResult.hasTargets()) return;

        inputs.nearestTagDistanceMeters = shooterCameraResult.getBestTarget().getBestCameraToTarget().getTranslation().getDistance(new Translation3d());

        Optional<EstimatedRobotPose> poseEstimatorResult = photonPoseEstimator.update();
        if (poseEstimatorResult.isEmpty()) return;
        
        Pose2d estimatedPose2d = poseEstimatorResult.get().estimatedPose.toPose2d();

        
        Logger.recordOutput("vision/robotPose3d", estimatedPose2d);
        Logger.recordOutput("vision/multiTagPoseAmbiguity", shooterCameraResult.getMultiTagResult().estimatedPose.ambiguity);


        //either use multitag or
        //if there's only one tag on the screen, only trust it if its pose ambiguity is below threshold
        //photonPoseEstimator automatically switches techniques when detecting different number of tags
        if (shooterCameraResult.targets.size() > 1 || (shooterCameraResult.targets.size() == 1 && shooterCameraResult.getBestTarget().getPoseAmbiguity() < 0.2)) {
            inputs.robotFieldPose = estimatedPose2d;
            inputs.timestampSeconds = poseEstimatorResult.get().timestampSeconds;
        }

    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {

        updateShooterCamera(inputs);

        PhotonPipelineResult intakeCameraResult = intakeCamera.getLatestResult();
        if (!intakeCameraResult.hasTargets()) {
            inputs.intakeSeesNote = false;
            inputs.nearestNoteYawDegrees = 0;
        }
        else {
            inputs.intakeSeesNote = true;
            inputs.nearestNoteYawDegrees = -intakeCameraResult.getBestTarget().getYaw();
        }
        
    };
}
