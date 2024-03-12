
package frc.robot.subsystems.drivetrain;

import java.util.function.Supplier;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
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
import edu.wpi.first.wpilibj2.command.Command;
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

    private SwerveDrivePoseEstimator fusedPoseEstimator;
    private SwerveDrivePoseEstimator wheelsOnlyPoseEstimator;

    public boolean isTrackingNote = false;
    public boolean isTrackingSpeakerInAuto = false;

    /** error measured in degrees, output is in degrees per second. */
    private PIDController angleController;

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

        //corresponds to x, y, and rotation standard deviations (meters and radians)
        Matrix<N3, N1> stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.005);

        
        //corresponds to x, y, and rotation standard deviations (meters and radians)
        //these values are automatically recalculated periodically depending on distance
        Matrix<N3, N1> visionStdDevs = VecBuilder.fill(0., 0., 0.);

        fusedPoseEstimator = new SwerveDrivePoseEstimator(
            DrivetrainConstants.swerveKinematics, 
            gyroInputs.robotYawRotation2d,
            getModulePositions(),
            new Pose2d(),
            stateStdDevs,
            visionStdDevs
        );

        wheelsOnlyPoseEstimator = new SwerveDrivePoseEstimator(
            DrivetrainConstants.swerveKinematics,
            gyroInputs.robotYawRotation2d,
            getModulePositions(), 
            new Pose2d());

        angleController = new PIDController(6, 0, 0.);
        angleController.enableContinuousInput(-180, 180);
        angleController.setTolerance(2.0); // degrees. TODO: could be more precise? Calculate based on margin for error at range?
    }


    /**
     * Drives the robot based on a desired ChassisSpeeds.
     * <p>
     * Takes in a robot relative ChassisSpeeds. Field relative control can be accomplished by using the ChassisSpeeds.fromFieldRelative() method.
     * @param desiredChassisSpeeds - Robot relative ChassisSpeeds object in meters per second and radians per second.
     * @param closedLoop - Whether or not to used closed loop PID control to control the speed of the drive wheels.
    */
    public void robotOrientedDrive(ChassisSpeeds desiredChassisSpeeds, boolean closedLoop) {
        SwerveModuleState[] swerveModuleStates = DrivetrainConstants.swerveKinematics.toSwerveModuleStates(desiredChassisSpeeds);

        setModuleStates(swerveModuleStates, closedLoop);
    }


    /**
     * TODO: Documentation
     * @param desiredChassisSpeeds
     * @param closedLoop
     */
    public void fieldOrientedDrive(ChassisSpeeds desiredChassisSpeeds, boolean closedLoop) {
        Rotation2d currentOrientation = fusedPoseEstimator.getEstimatedPosition().getRotation();
        ChassisSpeeds robotOrientedSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(desiredChassisSpeeds, currentOrientation);
        this.robotOrientedDrive(robotOrientedSpeeds, closedLoop);
    }


    /**
     * TODO: documentation
     * @param desiredTranslationalSpeeds
     * @param desiredAngleDegrees
     */
    public void fieldOrientedDriveWhileAiming(ChassisSpeeds desiredTranslationalSpeeds, Rotation2d desiredAngle) {
        // Use PID controller to generate a desired angular velocity based on the desired angle
        double measuredAngle = fusedPoseEstimator.getEstimatedPosition().getRotation().getDegrees();
        double desiredAngleDegrees = desiredAngle.getDegrees();
        double desiredDegreesPerSecond = angleController.calculate(measuredAngle, desiredAngleDegrees);
        double desiredRadiansPerSecond = Math.toRadians(desiredDegreesPerSecond);

        ChassisSpeeds desiredSpeeds = new ChassisSpeeds();
        desiredSpeeds.vxMetersPerSecond = desiredTranslationalSpeeds.vxMetersPerSecond;
        desiredSpeeds.vyMetersPerSecond = desiredTranslationalSpeeds.vyMetersPerSecond;
        desiredSpeeds.omegaRadiansPerSecond = desiredRadiansPerSecond;
        this.fieldOrientedDrive(desiredSpeeds, true);
    }


    public Command fieldOrientedDriveCommand(Supplier<ChassisSpeeds> sourceOfDesiredSpeeds) {
        return this.run(() -> {
            ChassisSpeeds desiredSpeeds = sourceOfDesiredSpeeds.get();
            this.fieldOrientedDrive(desiredSpeeds, true);
        });
    }


    //could be used for a drivetrain command in the future; leave this as its own function
    public void setModuleStates(SwerveModuleState[] desiredStates, boolean closedLoop) {
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
        fusedPoseEstimator.resetPosition(gyroInputs.robotYawRotation2d, getModulePositions(), pose);
        wheelsOnlyPoseEstimator.resetPosition(gyroInputs.robotYawRotation2d, getModulePositions(), pose);
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
     * TODO: OUT OF DATE DOCUMENTATION?
     */ 
    public Pose2d getPoseMeters() {
        return fusedPoseEstimator.getEstimatedPosition();
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
     */
    private Matrix<N3, N1> getVisionStdDevs(double distToTargetMeters) {

        double slopeStdDevMetersPerMeter = .5;
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

        Translation2d location = fusedPoseEstimator.getEstimatedPosition().getTranslation();

        setPoseMeters(new Pose2d(location, rotation));
    }

    public double getAngleError() {
        if (isTrackingSpeakerInAuto) {
            return getPoseMeters().getRotation().minus(getAngleFromDriveToSpeaker()).getDegrees();
        }
        return angleController.getPositionError();
    }

    public boolean intakeSeesNote() {
        return visionInputs.intakeSeesNote;
    }

    /**
     * Gets the angle that the robot needs to aim at in order to intake the nearest ring
     * seen on the intake camera. This is used for the rotation override during auto.
     */
    public Rotation2d getFieldRelativeRotationToNote() {
        Rotation2d currentAngle = fusedPoseEstimator.getEstimatedPosition().getRotation();
        return currentAngle.plus(Rotation2d.fromDegrees(visionInputs.nearestNoteYawDegrees));
    }

    public Optional<Rotation2d> getAutoRotationOverride() {
        if (isTrackingSpeakerInAuto) {
            return Optional.of(getAngleFromDriveToSpeaker());
        }
        if (isTrackingNote && visionInputs.intakeSeesNote) {
            return Optional.of(getFieldRelativeRotationToNote());
        }
        else {
            return Optional.empty();
        }
    }

    public boolean isAligned() {
        return angleController.atSetpoint();
    }

    private void updatePoseEstimator() {

        fusedPoseEstimator.update(gyroInputs.robotYawRotation2d, getModulePositions());
        wheelsOnlyPoseEstimator.update(gyroInputs.robotYawRotation2d, getModulePositions());


        Translation2d visionTranslation = visionInputs.robotFieldPose.getTranslation();
        Translation2d estimatedTranslation = fusedPoseEstimator.getEstimatedPosition().getTranslation();

        // don't add vision measurements that are too far away
        // for reference: it is 6 meters from speaker tags to wing.
        if (visionTranslation.getDistance(estimatedTranslation) < 2 && visionInputs.nearestTagDistanceMeters < 3) {
            fusedPoseEstimator.addVisionMeasurement(
                visionInputs.robotFieldPose, 
                visionInputs.timestampSeconds, 
                getVisionStdDevs(visionInputs.nearestTagDistanceMeters)
            );
        }
    }
        public Translation2d getSpeakerLocation() {
        return DriverStation.getAlliance().get() == Alliance.Red ?
               FieldConstants.redSpeakerTranslation2d :
               FieldConstants.blueSpeakerTranslation2d;
    }


    /** Calculates the horizontal translational distance from the center of the robot to the center of the front edge of the speaker.*/
    public double driveDistToSpeakerBaseMeters() {
        Translation2d speakerLocation = this.getSpeakerLocation();
        Translation2d robotLocation = this.getPoseMeters().getTranslation();
        return robotLocation.getDistance(speakerLocation);
    }

    public double armDistToSpeakerBaseMeters() {
        return this.driveDistToSpeakerBaseMeters() + FieldConstants.pivotOffsetMeters;
    }

    /** Calculates the angle the arm would aim at to make a straight line to the speaker target. */
    public double getSimpleArmDesiredDegrees() {
        double horizontalDistance = this.armDistToSpeakerBaseMeters();
        double verticalDistance = FieldConstants.speakerHeightMeters - FieldConstants.pivotHeightMeters;
        double radians = Math.atan2(verticalDistance, horizontalDistance); // prob don't need arctan2 here, regular arctan will do.
        return Math.toDegrees(radians);
    }

    /**
     * Gets the angle that the shooter needs to aim at in order for a note to make it into the speaker.
     * This accounts for distance and gravity.
     * @return - Angle in degrees, with 0 being straight forward and a positive angle being pointed upwards.
    */
    public double getGravCompensatedArmDesiredDegrees(double exitVelocityMetersPerSecond) {
        
        //see https://www.desmos.com/calculator/czxwosgvbz

        double h = FieldConstants.speakerHeightMeters-FieldConstants.pivotHeightMeters;
        double d = this.armDistToSpeakerBaseMeters();
        double v = exitVelocityMetersPerSecond;
        double g = 9.81;

        double a = (h*h)/(d*d)+1;
        double b = -2*(h*h)*(v*v)/(d*d) - (v*v) - g*h;
        double c = (h*h)*Math.pow(v, 4)/(d*d) + (g*g)*(d*d)/4 + g*h*(v*v);

        double vy = Math.sqrt((-b-Math.sqrt(b*b-4*a*c))/(2*a));

        return Math.toDegrees(Math.asin(vy/v));
    }

    /** TODO: documentation */
    public Rotation2d getAngleFromDriveToSpeaker() {
        Translation2d speakerLocation = null;

        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (!alliance.isPresent()) {
            // Don't turn if we can't tell where to aim
            return getPoseMeters().getRotation();
        }

        if (alliance.get() == Alliance.Blue) {
            speakerLocation = FieldConstants.blueSpeakerTranslation2d;
        }
        else if (alliance.get() == Alliance.Red) {
            speakerLocation = FieldConstants.redSpeakerTranslation2d;
        }

        // We want robot to align with the vector from robot to speaker
        // that means we want the robot's angle to be the same as that vector's angle
        Translation2d vector = speakerLocation.minus(getPoseMeters().getTranslation());
        return vector.getAngle();
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

        updatePoseEstimator();


        Logger.recordOutput("drivetrain/fusedPose", fusedPoseEstimator.getEstimatedPosition());
        Logger.recordOutput("drivetrain/wheelsOnlyPose", wheelsOnlyPoseEstimator.getEstimatedPosition());

        Logger.recordOutput(
            "drivetrain/swerveModuleStates",
            new SwerveModuleState[] {
              swerveModules[0].getState(),
              swerveModules[1].getState(),
              swerveModules[2].getState(),
              swerveModules[3].getState()
          });

        Logger.recordOutput("drivetrain/anglePIDSetpoint", Rotation2d.fromDegrees(angleController.getSetpoint()));
        Logger.recordOutput("drivetrain/isAligned", isAligned());
        Logger.recordOutput("drivetrain/isTrackingNote", isTrackingNote);
        
    }
}
