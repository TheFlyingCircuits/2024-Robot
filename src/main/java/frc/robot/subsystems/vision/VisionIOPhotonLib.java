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
    PhotonCamera trapCamera;
    PhotonCamera noteCamera;
    PhotonPoseEstimator shooterPoseEstimator;
    PhotonPoseEstimator trapPoseEstimator;

    public VisionIOPhotonLib() {
        shooterCamera = new PhotonCamera("frontCamera");
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
            shooterCamera,
            VisionConstants.robotToTrapCamera
        );
    }


    private void updateShooterCamera(VisionIOInputs inputs) {
        PhotonPipelineResult shooterCameraResult = shooterCamera.getLatestResult();
        if (!shooterCameraResult.hasTargets()) return;

        inputs.nearestTagDistanceMeters = shooterCameraResult.getBestTarget().getBestCameraToTarget().getTranslation().getDistance(new Translation3d());

        Optional<EstimatedRobotPose> poseEstimatorResult = shooterPoseEstimator.update();
        if (poseEstimatorResult.isEmpty()) return;
        
        Pose2d estimatedPose2d = poseEstimatorResult.get().estimatedPose.toPose2d();

        //either use multitag or
        //if there's only one tag on the screen, only trust it if its pose ambiguity is below threshold
        //photonPoseEstimator automatically switches techniques when detecting different number of tags
        if (shooterCameraResult.targets.size() > 1 || (shooterCameraResult.targets.size() == 1 && shooterCameraResult.getBestTarget().getPoseAmbiguity() < 0.2)) {
            inputs.robotFieldPose = estimatedPose2d;
            inputs.timestampSeconds = poseEstimatorResult.get().timestampSeconds;
        }
    }

    private void updateTrapCamera(VisionIOInputs inputs) {
        PhotonPipelineResult trapCameraResult = trapCamera.getLatestResult();
        if (!trapCameraResult.hasTargets()) return;

        inputs.nearestTagDistanceMeters = trapCameraResult.getBestTarget().getBestCameraToTarget().getTranslation().getDistance(new Translation3d());

        Optional<EstimatedRobotPose> poseEstimatorResult = trapPoseEstimator.update();
        if (poseEstimatorResult.isEmpty()) return;
        
        Pose2d estimatedPose2d = poseEstimatorResult.get().estimatedPose.toPose2d();
        //either use multitag or
        //if there's only one tag on the screen, only trust it if its pose ambiguity is below threshold
        //photonPoseEstimator automatically switches techniques when detecting different number of tags
        if (trapCameraResult.targets.size() > 1 || (trapCameraResult.targets.size() == 1 && trapCameraResult.getBestTarget().getPoseAmbiguity() < 0.2)) {
            inputs.robotFieldPose = estimatedPose2d;
            inputs.timestampSeconds = poseEstimatorResult.get().timestampSeconds;
        }
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {

        updateShooterCamera(inputs);
        updateTrapCamera(inputs);

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
