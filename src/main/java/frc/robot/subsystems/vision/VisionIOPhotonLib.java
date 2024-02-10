package frc.robot.subsystems.vision;

import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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

        if (result.hasTargets()) {
            inputs.nearestTagDistanceMeters = result.getBestTarget().getBestCameraToTarget().getTranslation().getDistance(new Translation3d());
        };


        Optional<EstimatedRobotPose> poseEstimatorResult = photonPoseEstimator.update();
        if (poseEstimatorResult.isEmpty()) return;

        Pose2d estimatedPose2d = poseEstimatorResult.get().estimatedPose.toPose2d();

        //TODO: flip pose depending on team

        Logger.recordOutput("vision/robotPose3d", estimatedPose2d);

        inputs.robotFieldPose = estimatedPose2d;
        inputs.timestampSeconds = poseEstimatorResult.get().timestampSeconds;
    };
}
