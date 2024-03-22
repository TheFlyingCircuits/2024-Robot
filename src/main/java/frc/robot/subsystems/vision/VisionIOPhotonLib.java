package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;


import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
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
            slopeStdDevMetersPerMeterX = 0.004;
            slopeStdDevMetersPerMeterY = 0.009;
        }

        else {
            slopeStdDevMetersPerMeterX = 0.008;
            slopeStdDevMetersPerMeterY = 0.008;
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
        if (!pipelineResult.hasTargets())
            return Optional.empty();

        Optional<EstimatedRobotPose> poseEstimatorResult = estimator.update();
        if (poseEstimatorResult.isEmpty())
            return Optional.empty();

        //either use multitag or
        //if there's only one tag on the screen, only trust it if its pose ambiguity is below threshold
        //photonPoseEstimator automatically switches techniques when detecting different number of tags
        if (pipelineResult.targets.size() == 1 && pipelineResult.getBestTarget().getPoseAmbiguity() > 0.2)
            return Optional.empty();

        output.nearestTagDistanceMeters = pipelineResult.getBestTarget().getBestCameraToTarget().getTranslation().getDistance(new Translation3d());
        
        if (output.nearestTagDistanceMeters > 5)
            return Optional.empty();


        output.robotFieldPose = poseEstimatorResult.get().estimatedPose.toPose2d();
        output.timestampSeconds = poseEstimatorResult.get().timestampSeconds;
        output.stdDevs = getVisionStdDevs(output.nearestTagDistanceMeters, (pipelineResult.targets.size() > 1));  //different standard devs for different methods of detecting apriltags

        return Optional.of(output);
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        inputs.visionMeasurements = new ArrayList<VisionMeasurement>();
        
        Optional<VisionMeasurement> shooterResult = updateTagCamera(shooterCamera, shooterPoseEstimator);
        if (shooterResult.isPresent())
            inputs.visionMeasurements.add(shooterResult.get());

        Optional<VisionMeasurement> trapResult = updateTagCamera(trapCamera, trapPoseEstimator);
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
            double nearestNoteYawDegrees = -noteCameraResult.getBestTarget().getYaw();
            double notePitchDegrees = -noteCameraResult.getBestTarget().getPitch();


            //rotation to rotate a unit X vector in the camera frame to point at the note
            Rotation3d cameraToNoteRotation = new Rotation3d(
                0,
                Math.toRadians(notePitchDegrees),
                Math.toRadians(nearestNoteYawDegrees));

            //unit vector that points at the note, by rotating the unit vector in the camera frame by the rotation
            Vector<N3> unitTowardsNote = new Translation3d(1, 0, 0)
                .rotateBy(cameraToNoteRotation).toVector();

            //height of the camera above the note
            double h = VisionConstants.robotToNoteCamera.getZ();

            //pitch of the camera (positive is counterclockwise about the robot y axis)
            double cameraPitchRadians = VisionConstants.robotToNoteCamera.getRotation().getY();

            //distance from the camera to the floor (pointed straight out of the camera)
            double d = Units.inchesToMeters(30);//h/Math.sin(cameraPitchRadians);

            //vector pointing out of the camera and ending on the floor
            Vector<N3> anchorPoint = VecBuilder.fill(d, 0, 0);


            //normal vector of the floor in the camera frame, done by rotating
            //a negative unit x vector by the camera's pitch
            Vector<N3> floorNormal = 
                new Translation3d(-1, 0, 0)
                    .rotateBy(
                        new Rotation3d(0, Math.PI/2-cameraPitchRadians, 0)).toVector();
                        
            //distance from the camera to the note, in full 3d
            double t = anchorPoint.dot(floorNormal)/unitTowardsNote.dot(floorNormal);

            Translation3d cameraToNoteTranslation = new Translation3d(unitTowardsNote.times(t));

            //transform to bring a vector in the note camera frame into the robot frame
            Transform3d noteCameraToRobot = VisionConstants.robotToNoteCamera.inverse();

            //position of the note relative to the robot
            inputs.nearestNote = cameraToNoteTranslation.rotateBy(noteCameraToRobot.getRotation())
                .plus(noteCameraToRobot.getTranslation());

            //inputs.nearestNote = (new Pose3d(cameraToNoteTranslation, new Rotation3d())).transformBy(noteCameraToRobot).getTranslation();
        }
    };
}
