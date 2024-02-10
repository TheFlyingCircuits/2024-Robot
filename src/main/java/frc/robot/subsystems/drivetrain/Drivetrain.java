
package frc.robot.subsystems.drivetrain;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.constraint.SwerveDriveKinematicsConstraint;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.SwerveModuleConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOInputsAutoLogged;

public class Drivetrain extends SubsystemBase {

    private GyroIO gyroIO;
    private GyroIOInputsAutoLogged gyroInputs;

    private VisionIO visionIO;
    private VisionIOInputsAutoLogged visionInputs;

    private SwerveModule[] swerveModules;

    private SlewRateLimiter chassisSpeedsXSlewLimiter;
    private SlewRateLimiter chassisSpeedsYSlewLimiter;

    private SwerveDrivePoseEstimator poseEstimator;

    private Pose2d poseMeters;

    public Drivetrain(
        GyroIO gyroIO, 
        SwerveModuleIO flSwerveModuleIO, 
        SwerveModuleIO frSwerveModuleIO, 
        SwerveModuleIO blSwerveModuleIO, 
        SwerveModuleIO brSwerveModuleIO,
        VisionIO visionIO
    ) {

        this.gyroIO = gyroIO;
        gyroInputs = new GyroIOInputsAutoLogged();

        this.visionIO = visionIO;
        visionInputs = new VisionIOInputsAutoLogged();

        swerveModules = new SwerveModule[] {
            new SwerveModule(flSwerveModuleIO, 0),
            new SwerveModule(frSwerveModuleIO, 1),
            new SwerveModule(blSwerveModuleIO, 2),
            new SwerveModule(brSwerveModuleIO, 3)
        };

        gyroIO.setRobotYaw(0);

        poseMeters = new Pose2d(new Translation2d(0, 0), new Rotation2d(0));



        Matrix<N3, N1> stateStdDevs = new Matrix(Nat.N3(), Nat.N1());
        //corresponds to x, y, and rotation standard deviations (meters and radians)
        stateStdDevs.set(0, 0, 0.1);
        stateStdDevs.set(1, 0, 0.1);
        stateStdDevs.set(2, 0, 0.005);

        Matrix<N3, N1> visionStdDevs = new Matrix(Nat.N3(), Nat.N1());
        //corresponds to x, y, and rotation standard deviations (meters and radians)
        //these values are automatically recalculated periodically depending on distance
        visionStdDevs.set(0, 0, 0.0);
        visionStdDevs.set(1, 0, 0.0);
        visionStdDevs.set(2, 0, 0.);

        poseEstimator = new SwerveDrivePoseEstimator(
            DrivetrainConstants.swerveKinematics, 
            gyroInputs.robotYawRotation2d,
            getModulePositions(),
            poseMeters,
            stateStdDevs,
            visionStdDevs
        );

        chassisSpeedsXSlewLimiter = new SlewRateLimiter(DrivetrainConstants.maxDesiredTeleopAccelMetersPerSecondSquared);
        chassisSpeedsYSlewLimiter = new SlewRateLimiter(DrivetrainConstants.maxDesiredTeleopAccelMetersPerSecondSquared);
    }

    /**
     * Sets the robot pose's angle to the rotation passed into it.. This affects all subsequent uses of the robot's angle.
     * <br>
     * This can be used to set the direction the robot is currently facing to the 'forwards' direction.
     */
    public void setRobotRotation2d(Rotation2d rotation2d) {
        setPoseMeters(new Pose2d(getTranslationMeters(), rotation2d));
    }

    /**
     * Gets the angle of the robot measured by the gyroscope as a Rotation2d (continuous).
     * @return rotation2d - this angle will be counterclockwise positive.
     */
    public Rotation2d getRobotRotation2d() {
        return poseMeters.getRotation();
    }

    /**
     * Drives the robot based on a desired ChassisSpeeds.
     * <p>
     * Takes in a robot relative ChassisSpeeds. Field relative control can be accomplished by using the ChassisSpeeds.fromFieldRelative() method.
     * @param desiredChassisSpeeds - Robot relative ChassisSpeeds object in meters per second and radians per second.
    */
    public void drive(ChassisSpeeds desiredChassisSpeeds) {
        drive(desiredChassisSpeeds, true);
    }

    /**
     * Drives the robot based on a desired ChassisSpeeds.
     * <p>
     * Takes in a robot relative ChassisSpeeds. Field relative control can be accomplished by using the ChassisSpeeds.fromFieldRelative() method.
     * @param desiredChassisSpeeds - Robot relative ChassisSpeeds object in meters per second and radians per second.
    */
    public void drive(ChassisSpeeds desiredChassisSpeeds, boolean closedLoop) {

        desiredChassisSpeeds.vxMetersPerSecond = chassisSpeedsXSlewLimiter.calculate(desiredChassisSpeeds.vxMetersPerSecond);
        desiredChassisSpeeds.vyMetersPerSecond = chassisSpeedsYSlewLimiter.calculate(desiredChassisSpeeds.vyMetersPerSecond);

        SwerveModuleState[] swerveModuleStates = DrivetrainConstants.swerveKinematics.toSwerveModuleStates(desiredChassisSpeeds);

        if (closedLoop) {
            setModuleStatesClosedLoop(swerveModuleStates);
        }
        else {
            setModuleStatesOpenLoop(swerveModuleStates);
        }
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStatesClosedLoop(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DrivetrainConstants.maxAchievableVelocityMetersPerSecond);
        for (SwerveModule mod : swerveModules) {
            mod.setDesiredState(desiredStates[mod.moduleIndex], true);
        }
    }

    public void setModuleStatesClosedLoopNoOptimize(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DrivetrainConstants.maxAchievableVelocityMetersPerSecond);
        for (SwerveModule mod : swerveModules) {
            mod.setDesiredStateNoOptimize(desiredStates[mod.moduleIndex], true);
        }
    }


  //useful for debugging
    public void setModuleStatesOpenLoop(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DrivetrainConstants.maxAchievableVelocityMetersPerSecond);
        for (SwerveModule mod : swerveModules) {
            mod.setDesiredState(desiredStates[mod.moduleIndex], false);
        }
    }


    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] swervePositions = new SwerveModulePosition[4];

        for (SwerveModule mod : swerveModules) {
            swervePositions[mod.moduleIndex] = mod.getPosition();
        }

        return swervePositions;
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] swerveStates = new SwerveModuleState[4];

        for (SwerveModule mod : swerveModules) {
            swerveStates[mod.moduleIndex] = mod.getState();
        }

        return swerveStates;
    }

    public ChassisSpeeds getChassisSpeeds() {
        return DrivetrainConstants.swerveKinematics.toChassisSpeeds(getModuleStates());
    }


    /**
     * Sets the current position of the robot on the field in meters.
     * <p>
     * A positive X value brings the robot towards the opposing alliance,
     * and a positive Y value brings the robot left as viewed by your alliance.
     * @param pose
     */
    public void setPoseMeters(Pose2d pose) {
        poseEstimator.resetPosition(gyroInputs.robotYawRotation2d, getModulePositions(), pose);
        poseMeters = pose;
    }

    /**
     * Gets the current position of the robot on the field in meters, 
     * based off of our odometry and vision estimation.
     * This value considers the origin to be the right side of the robot's current alliance.
     * <p>
     * A positive X value brings the robot towards the opposing alliance,
     * and a positive Y value brings the robot left as viewed by your alliance.
     * @return The current position of the robot on the field in meters.
     */ 
    public Pose2d getPoseMeters() {
        return poseMeters;
    }

    public Translation2d getTranslationMeters() {
        return poseMeters.getTranslation();
    }

    /**
     * Takes the estimated pose from the vision, and sets our current poseEstimator pose to this one.
     */
    public void setPoseToVisionMeasurement() {
        setPoseMeters(visionInputs.robotFieldPose);
    }

    /**
     * Calculates a matrix of standard deviations of the vision pose estimate, in meters and degrees. 
     * This is a function of the distance from the camera to the april tag.
     * @param distToTargetMeters - Distance from the camera to the apriltag. 
     * @return
     */
    private Matrix<N3, N1> getVisionStdDevs(double distToTargetMeters) {

        // Matrix<N3, N1> visionStdDevs = new Matrix(Nat.N3(), Nat.N1());
        //corresponds to x, y, and rotation standard deviations (meters and radians)
        //these values are automatically recalculated periodically depending on distance


        //large drop off in reliability after distance is greater than 3.5 meters, so we are interpreting as linear before then
        // if (distToTargetMeters <= 3.5) {
        //     visionStdDevs.set(0, 0, -0.0182 + 0.00996*distToTargetMeters);
        //     visionStdDevs.set(1, 0, -0.0196 + 0.0103*distToTargetMeters);
        //     visionStdDevs.set(2, 0, -0.0078 + 0.00438);
        // }

        double slopeStdDevMetersPerMeter = 2.;
        double slopeStdDevRadiansPerMeter = 3;

        

        return VecBuilder.fill(
            slopeStdDevMetersPerMeter*distToTargetMeters,
            slopeStdDevMetersPerMeter*distToTargetMeters,
            slopeStdDevRadiansPerMeter*distToTargetMeters
        );
    }


    @Override
    public void periodic() {
        gyroIO.updateInputs(gyroInputs);
        visionIO.updateInputs(visionInputs);
        for (SwerveModule mod : swerveModules)
            mod.periodic();
        Logger.processInputs("gyroInputs", gyroInputs);
        Logger.processInputs("visionInputs", visionInputs);


        poseMeters = poseEstimator.update(gyroInputs.robotYawRotation2d, getModulePositions());
        poseEstimator.addVisionMeasurement(
            visionInputs.robotFieldPose, 
            visionInputs.timestampSeconds, 
            getVisionStdDevs(visionInputs.nearestTagDistanceMeters)
        );

        Logger.recordOutput("drivetrain/swerveOdometry", getPoseMeters());
        Logger.recordOutput("drivetrain/poseEstimatorPose", poseEstimator.getEstimatedPosition());

        Logger.recordOutput(
            "drivetrain/swerveModuleStates",
            new SwerveModuleState[] {
              swerveModules[0].getState(),
              swerveModules[1].getState(),
              swerveModules[2].getState(),
              swerveModules[3].getState()
          });

    }
}
