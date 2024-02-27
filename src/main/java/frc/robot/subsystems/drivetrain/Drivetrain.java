
package frc.robot.subsystems.drivetrain;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.FieldConstants;
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

        //corresponds to x, y, and rotation standard deviations (meters and radians)
        Matrix<N3, N1> stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.005);

        
        //corresponds to x, y, and rotation standard deviations (meters and radians)
        //these values are automatically recalculated periodically depending on distance
        Matrix<N3, N1> visionStdDevs = VecBuilder.fill(0., 0., 0.);

        poseEstimator = new SwerveDrivePoseEstimator(
            DrivetrainConstants.swerveKinematics, 
            gyroInputs.robotYawRotation2d,
            getModulePositions(),
            poseMeters,
            stateStdDevs,
            visionStdDevs
        );

        // TODO: single slew limiter for total speed instead of seperate for x and y
        chassisSpeedsXSlewLimiter = new SlewRateLimiter(DrivetrainConstants.maxDesiredTeleopAccelMetersPerSecondSquared);
        chassisSpeedsYSlewLimiter = new SlewRateLimiter(DrivetrainConstants.maxDesiredTeleopAccelMetersPerSecondSquared);
    }

    /**
     * Sets the robot pose's angle to the rotation passed into it.. This affects all subsequent uses of the robot's angle.
     * You should be setting an angle of 0 to be facing away from the blue alliance wall.
     * <br>
     * This can be used to set the direction the robot is currently facing to the 'forwards' direction.
     */
    public void setRobotRotation2d(Rotation2d rotation2d) {
        setPoseMeters(new Pose2d(getPoseMeters().getTranslation(), rotation2d));
    }

    /**
     * Gets the angle of the robot measured by the gyroscope as a Rotation2d (continuous).
     * An angle of 0 is always facing away from the blue alliance wall.
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
     * @param closedLoop - Whether or not to used closed loop PID control to control the speed of the drive wheels.
    */
    public void drive(ChassisSpeeds desiredChassisSpeeds, boolean closedLoop) {

        desiredChassisSpeeds.vxMetersPerSecond = chassisSpeedsXSlewLimiter.calculate(desiredChassisSpeeds.vxMetersPerSecond);
        desiredChassisSpeeds.vyMetersPerSecond = chassisSpeedsYSlewLimiter.calculate(desiredChassisSpeeds.vyMetersPerSecond);

        
        SwerveModuleState[] swerveModuleStates = DrivetrainConstants.swerveKinematics.toSwerveModuleStates(desiredChassisSpeeds);

        setModuleStates(swerveModuleStates, closedLoop);
    }

    //could be used for a drivetrain command in the future; leave this as its own function
    public void setModuleStates(SwerveModuleState[] desiredStates, boolean closedLoop) {
        // TODO: why saturate based on theory instead of emperical measurements?
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DrivetrainConstants.maxAchievableVelocityMetersPerSecond);
        for (SwerveModule mod : swerveModules) {
            mod.setDesiredState(desiredStates[mod.moduleIndex], closedLoop);
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

    /** Gets robot relative chassis speeds. */
    public ChassisSpeeds getChassisSpeeds() {
        return DrivetrainConstants.swerveKinematics.toChassisSpeeds(getModuleStates());
    }


    /**
     * Sets the current position of the robot on the field in meters.
     * <p>
     * A positive X value brings the robot towards the opposing alliance,
     * and a positive Y value brings the robot left as viewed by your alliance.
     * Rotations are counter-clockwise positive, with an angle of 0 facing away from the blue alliance wall.
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
     * Rotations are counter-clockwise positive, with an angle of 0 facing away from the blue alliance wall.
     * @return The current position of the robot on the field in meters.
     */ 
    public Pose2d getPoseMeters() {
        return poseMeters;
    }

    /** Calculates the horizontal translational distance from the center of the robot to the center of the front edge of the speaker.*/
    public double distToSpeakerBaseMeters() {
        return getPoseMeters().getTranslation().getDistance(
            DriverStation.getAlliance().get() == Alliance.Red ? 
                FieldConstants.redSpeakerTranslation2d : FieldConstants.blueSpeakerTranslation2d
        );
    }

    /** 
     * Gets the angle the robot needs to aim in order for the shooter to 
     * point at the center of the front edge of your alliance's speaker. 
     * This angle is counter-clockwise positive with an angle of zero facing away from the blue alliance wall.
     * */
    public Rotation2d getAngleToSpeaker() {
        return (DriverStation.getAlliance().get() == Alliance.Red ? 
                FieldConstants.redSpeakerTranslation2d : FieldConstants.blueSpeakerTranslation2d)
                    .minus(getPoseMeters().getTranslation()).getAngle();
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

        double slopeStdDevMetersPerMeter = 2.;
        double slopeStdDevRadiansPerMeter = 1000;

        //corresponds to x, y, and rotation standard deviations (meters and radians)
        return VecBuilder.fill(
            slopeStdDevMetersPerMeter*distToTargetMeters,
            slopeStdDevMetersPerMeter*distToTargetMeters,
            slopeStdDevRadiansPerMeter*distToTargetMeters
        );
    }


    /**
     * Sets the angle of the robot's pose so that it is facing forward, away from your alliance wall. 
     * This allows the driver to realign the drive direction and other calls to our angle.
     */
    public void setRobotFacingForward() {
        Rotation2d rotation = (DriverStation.getAlliance().get() == Alliance.Blue) ?
            Rotation2d.fromDegrees(0) : Rotation2d.fromDegrees(180);

        setPoseMeters(new Pose2d(getPoseMeters().getTranslation(), rotation));
    }


    @Override
    public void periodic() {
        gyroIO.updateInputs(gyroInputs);
        visionIO.updateInputs(visionInputs);
        for (SwerveModule mod : swerveModules)
            mod.periodic();

        if (gyroIO instanceof GyroIOSim) //calculates sim gyro
            gyroIO.calculateYaw(getModulePositions());
          

        Logger.processInputs("gyroInputs", gyroInputs);
        Logger.processInputs("visionInputs", visionInputs);


        poseMeters = poseEstimator.update(gyroInputs.robotYawRotation2d, getModulePositions());
        poseEstimator.addVisionMeasurement(
            visionInputs.robotFieldPose, 
            visionInputs.timestampSeconds, 
            getVisionStdDevs(visionInputs.nearestTagDistanceMeters)
        );

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
