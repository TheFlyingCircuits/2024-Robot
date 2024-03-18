
package frc.robot.subsystems.drivetrain;

import java.util.function.Supplier;
import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.hardware.TalonFX;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
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
import frc.robot.Constants.FieldElement;
import frc.robot.Constants.VisionConstants;
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

    private static Orchestra orchestra;
    private String[] songs = {
        "overworld.chrp",
        "pokemon.chrp"
    };
    private String currentSong = "";
    private String lastSong = "";

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

        orchestra = new Orchestra();

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
        angleController.setTolerance(3.0); // degrees. TODO: could be more precise? Calculate based on margin for error at range?

        configPathPlanner();
    }

    private void configPathPlanner() {
        AutoBuilder.configureHolonomic(
            this::getPoseMeters, // Robot pose supplier
            (Pose2d dummy) -> {}, // Method to reset odometry (will be called if your auto has a starting pose)
                                  // Note: We never let PathPlanner set the pose, we always seed pose using cameras and apriltags.
            () -> {return DrivetrainConstants.swerveKinematics.toChassisSpeeds(getModuleStates());}, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (ChassisSpeeds speeds) -> {this.robotOrientedDrive(speeds, true);}, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(4.0, 0.0, 0.0), // Rotation PID constants // TODO: get from angleController? or do we want them to have different gains?
                    DrivetrainConstants.maxAchievableVelocityMetersPerSecond, // Max module speed, in m/s // TODO: be more conservative? This is a theoretical max that's higher than the actual max.
                    DrivetrainConstants.drivetrainRadiusMeters, // Drive base radius in meters. Distance from robot center to furthest module.
                    new ReplanningConfig() // Default path replanning config. See the API for the options here
            ),
            () -> {
              // Boolean supplier that controls when the path will be mirrored
              // We by default draw the paths on the red side of the field, mirroring them if we are on the blue alliance.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Blue;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
        );

        PPHolonomicDriveController.setRotationTargetOverride(this::getAutoRotationOverride);

        PathPlannerLogging.setLogActivePathCallback( (activePath) -> {
            Logger.recordOutput("PathPlanner/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });

        PathPlannerLogging.setLogTargetPoseCallback( (targetPose) -> {
            Logger.recordOutput("PathPlanner/TrajectorySetpoint", targetPose);
        });
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

        //commented values are empirically found
        double slopeStdDevMetersPerMeterX = 0.5;//0.001;
        double slopeStdDevMetersPerMeterY = 0.5;//0.003;
        // TODO: maybe just calculate based on last couple seconds instead of as a funciton of distance?

        double slopeStdDevRadiansPerMeter = 1000;

        //corresponds to x, y, and rotation standard deviations (meters and radians)
        return VecBuilder.fill(
            slopeStdDevMetersPerMeterX*distToTargetMeters,
            slopeStdDevMetersPerMeterY*distToTargetMeters,
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
            Rotation2d desiredAngle = getAngleFromDriveToFieldElement(FieldElement.SPEAKER);
            Rotation2d measuredAngle = getPoseMeters().getRotation();
            return measuredAngle.minus(desiredAngle).getDegrees();
            // TODO: add leds for not tracking too?
            // maybe see if we can pass in our own angle controller to path planner?
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
            return Optional.of(getAngleFromDriveToFieldElement(FieldElement.SPEAKER));
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
        if (visionTranslation.getDistance(estimatedTranslation) < 1. && visionInputs.nearestTagDistanceMeters < 4) {
            fusedPoseEstimator.addVisionMeasurement(
                visionInputs.robotFieldPose, 
                visionInputs.timestampSeconds, 
                getVisionStdDevs(visionInputs.nearestTagDistanceMeters)
            );
        }
    }


    /** Calculates the horizontal translational distance from the center of the robot to the central apriltag of the speaker.*/
    public double driveDistToSpeakerBaseMeters() {
        Translation2d speakerLocation = getLocationOfFieldElement(FieldElement.SPEAKER).getTranslation();
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

    /**
     * Computes the vector that points from the drivetrain's current location to the given target,
     * then returns the angle of that vector relative to the x axis of the field coordinate system.
     * @param targetLocationOnField The tip of the vector, as measured in the field coordinate system.
     * @return
     */
    public Rotation2d getAngleFromDriveToTarget(Translation2d targetLocationOnField) {
        Translation2d vector = targetLocationOnField.minus(getPoseMeters().getTranslation());
        return vector.getAngle();
    }

    public Rotation2d getAngleFromDriveToFieldElement(FieldElement element) {
        Pose2d pose = getLocationOfFieldElement(element);
        return getAngleFromDriveToTarget(pose.getTranslation());
    }

    public Pose2d getLocationOfFieldElement(FieldElement element) {

        Optional<Alliance> alliance = DriverStation.getAlliance();
        AprilTagFieldLayout fieldLayout = VisionConstants.aprilTagFieldLayout;

        if (alliance.isPresent() && alliance.get() == Alliance.Blue) {
            if (element == FieldElement.SPEAKER) { return fieldLayout.getTagPose(7).get().toPose2d(); }
            if (element == FieldElement.AMP) { return fieldLayout.getTagPose(6).get().toPose2d(); }
            if (element == FieldElement.STAGE_LEFT) { return fieldLayout.getTagPose(15).get().toPose2d(); }
            if (element == FieldElement.STAGE_RIGHT) { return fieldLayout.getTagPose(16).get().toPose2d(); }
            if (element == FieldElement.CENTER_STAGE) { return fieldLayout.getTagPose(14).get().toPose2d(); }
        }

        if (alliance.isPresent() && alliance.get() == Alliance.Red) {
            if (element == FieldElement.SPEAKER) { return fieldLayout.getTagPose(4).get().toPose2d(); }
            if (element == FieldElement.AMP) { return fieldLayout.getTagPose(5).get().toPose2d(); }
            if (element == FieldElement.STAGE_LEFT) { return fieldLayout.getTagPose(11).get().toPose2d(); }
            if (element == FieldElement.STAGE_RIGHT) { return fieldLayout.getTagPose(12).get().toPose2d(); }
            if (element == FieldElement.CENTER_STAGE) { return fieldLayout.getTagPose(13).get().toPose2d(); }
        }

        // If we don't have comms and can't tell what alliance we're on,
        // then just return the pose of the robot. This will make it so
        // when the robot tries to target the field element, it should
        // just stay in place, which seems like the safest thing to do.
        return getPoseMeters();
    }

    public void addInstrument(TalonFX kraken) {
        orchestra.addInstrument(kraken);
    }

    public static Orchestra getOrchestra() {
        return orchestra;
    }

    public StatusCode[] playOrchestra() {
        List<String> shuffledMusicFiles = new ArrayList<String>(List.of(songs));
        Collections.shuffle(shuffledMusicFiles);
        currentSong = shuffledMusicFiles.get(0);
        while(currentSong.equals(lastSong)) {
            System.out.println("prev song: " + lastSong
            +"\n== queued  : " + currentSong);
            Collections.shuffle(shuffledMusicFiles);
            currentSong = shuffledMusicFiles.get(0);
        };
        System.out.println("song queued: " + currentSong);
        lastSong = currentSong;

        StatusCode loadStatus = orchestra.loadMusic(currentSong);
        StatusCode playStatus = orchestra.play();
        StatusCode[] codes = {loadStatus, playStatus};
        return codes;
    }
    public StatusCode stopOrchestra() {
        StatusCode stopCode = orchestra.stop();
        return stopCode;
    }
    public boolean isSongPlaying() {
        return orchestra.isPlaying();
    }
    public int songTimestamp() {
        return ((int)orchestra.getCurrentTime());
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
