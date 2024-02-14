package frc.robot.subsystems.vision;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Constants.VisionConstants;

public class VisionIOPhotonLib implements VisionIO {
    
    PhotonCamera camera;
    PhotonPoseEstimator photonPoseEstimator;

    public VisionIOPhotonLib() {
        camera = new PhotonCamera("frontCamera");

        photonPoseEstimator = new PhotonPoseEstimator(
            VisionConstants.aprilTagFieldLayout, 
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            camera,
            VisionConstants.robotToCamera
        );
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {


        var result = camera.getLatestResult();
        if (!result.hasTargets()) return;

        inputs.nearestTagDistanceMeters = result.getBestTarget().getBestCameraToTarget().getTranslation().getDistance(new Translation3d());

        Optional<EstimatedRobotPose> poseEstimatorResult = photonPoseEstimator.update();
        if (poseEstimatorResult.isEmpty()) return;
        
        Pose2d estimatedPose2d = poseEstimatorResult.get().estimatedPose.toPose2d();

        
        Logger.recordOutput("vision/robotPose3d", estimatedPose2d);
        Logger.recordOutput("vision/multiTagPoseAmbiguity", result.getMultiTagResult().estimatedPose.ambiguity);


        //either use multitag or
        //if there's only one tag on the screen, only trust it if its pose ambiguity is below threshold
        //photonPoseEstimator automatically switches techniques when detecting different number of tags
        if (result.targets.size() > 1 || (result.targets.size() == 1 && result.getBestTarget().getPoseAmbiguity() < 0.2)) {
            inputs.robotFieldPose = estimatedPose2d;
            inputs.timestampSeconds = poseEstimatorResult.get().timestampSeconds;
        }
    };
}
