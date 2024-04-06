
package frc.robot.subsystems.drivetrain;

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

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.FlyingCircuitUtils;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.FieldElement;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIO.VisionIOInputsLogged;
import frc.robot.subsystems.vision.VisionIO.VisionMeasurement;

public class Drivetrain extends SubsystemBase {

    private GyroIO gyroIO;
    private GyroIOInputsAutoLogged gyroInputs;

    private VisionIO visionIO;
    private VisionIOInputsLogged visionInputs;

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

    /** error measured in meters, output is in meters per second. */
    private PIDController translationController;

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
        visionInputs = new VisionIOInputsLogged();

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

        angleController = new PIDController(11, 0, 0.5); // kP has units of degreesPerSecond per degree of error.
        angleController.enableContinuousInput(-180, 180);
        angleController.setTolerance(1.0); // degrees.

        translationController = new PIDController(4.0, 0, 0); // kP has units of metersPerSecond per meter of error.
        translationController.setTolerance(0.05); // 5 centimeters

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
                    new PIDConstants(3.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(4.0, 0.0, 0.0), // Rotation PID constants // These are different from our angleController gain(s), after testing.
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

        // Enable custom rotation targets during auto for note & speaker tracking
        PPHolonomicDriveController.setRotationTargetOverride(this::getAutoRotationOverride);

        // Register logging callbacks so that PathPlanner data shows up in advantage scope.
        PathPlannerLogging.setLogActivePathCallback( (activePath) -> {
            Logger.recordOutput("PathPlanner/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });

        PathPlannerLogging.setLogTargetPoseCallback( (targetPose) -> {
            // update the desired angle in the angle controller
            // this is only to allow the LEDs to show progress in auto.
            // The actual angle controller that sends commands in auto is the one from PathPlanner.
            double measuredAngleDegrees = getPoseMeters().getRotation().getDegrees();
            double desiredAngleDegrees = targetPose.getRotation().getDegrees();
            angleController.calculate(measuredAngleDegrees, desiredAngleDegrees);
            Logger.recordOutput("PathPlanner/TrajectorySetpoint", targetPose);
        });
    }


    //**************** DRIVING ****************/


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
     * Drives the robot at a desired chassis speeds. The coordinate system
     * is the same as the one as the one for setPoseMeters().
     * 
     * @param desiredChassisSpeeds - Field relative chassis speeds, in m/s and rad/s. 
     * @param closedLoop - Whether or not to drive the drive wheels with using feedback control.
     */
    public void fieldOrientedDrive(ChassisSpeeds desiredChassisSpeeds, boolean closedLoop) {
        Rotation2d currentOrientation = getPoseMeters().getRotation();
        ChassisSpeeds robotOrientedSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(desiredChassisSpeeds, currentOrientation);
        this.robotOrientedDrive(robotOrientedSpeeds, closedLoop);
    }


    /**
     * Drives the robot at a desired chassis speeds, while automatically aiming
     * at a rotation target. The coordinate system is the same as the one as the 
     * one for setPoseMeters().
     * 
     * @param desiredTranslationalSpeeds - Field relative chassis speeds, in m/s. The rotation speed target is not used. 
     * @param desiredAngle - Rotation2d of the target angle to aim at. This angle is CCW positive, with 0 
     * pointing away from the blue alliance wall.
     */
    public void fieldOrientedDriveWhileAiming(ChassisSpeeds desiredTranslationalSpeeds, Rotation2d desiredAngle) {
        // Use PID controller to generate a desired angular velocity based on the desired angle
        double measuredAngle = getPoseMeters().getRotation().getDegrees();
        double desiredAngleDegrees = desiredAngle.getDegrees();
        double desiredRadiansPerSecond = Math.toRadians(angleController.calculate(measuredAngle, desiredAngleDegrees));

        ChassisSpeeds desiredSpeeds = new ChassisSpeeds(
            desiredTranslationalSpeeds.vxMetersPerSecond,
            desiredTranslationalSpeeds.vyMetersPerSecond,
            desiredRadiansPerSecond
        );

        this.fieldOrientedDrive(desiredSpeeds, true);
    }

    /**
     * A drive function that's useful for aligning to different field elements.
     * The robot will snap to the line on the field that passes through
     * {@code lineToDriveOn.getTranslation()} and points in the direction of
     * {@code lineToDriveOn.getRotation()}. The driver can still move
     * the robot along this line.
     * @param rawSpeedRequest
     * @param lineToDriveOn
     */
    public void fieldOrientedDriveOnALine(ChassisSpeeds rawSpeedRequest, Pose2d lineToDriveOn) {
        // 0) Extract some data for calculations
        Translation2d pointOnLine = lineToDriveOn.getTranslation();
        Translation2d directionVectorAlongLine = new Translation2d(lineToDriveOn.getRotation().getCos(), lineToDriveOn.getRotation().getSin());

        // 1) Find the vector from the robot's current position on the field to a point on the line
        Translation2d vectorFromRobotToAnchor = pointOnLine.minus(getPoseMeters().getTranslation());

        // 2) Split this vector into 2 components, one along the line, and one perpendicular to the line
        double projectionOntoLine = vectorFromRobotToAnchor.getX() * directionVectorAlongLine.getX() + vectorFromRobotToAnchor.getY() * directionVectorAlongLine.getY();
        Translation2d componentAlongLine = directionVectorAlongLine.times(projectionOntoLine);
        Translation2d componentTowardsLine = vectorFromRobotToAnchor.minus(componentAlongLine);
        double distanceFromRobotToLine = componentTowardsLine.getNorm();

        // 3) Use a proportional controller to decide how quickly we should drive
        //    towards the line based on our perpendicular distance to the line.
        double speedTowardsLine = Math.abs(translationController.calculate(distanceFromRobotToLine, 0));

        // 4) Find the direction we should travel when driving at that speed
        ChassisSpeeds directionTowardsLine = new ChassisSpeeds();
        if (distanceFromRobotToLine > 0) {
            directionTowardsLine.vxMetersPerSecond = componentTowardsLine.getX() / distanceFromRobotToLine;
            directionTowardsLine.vyMetersPerSecond = componentTowardsLine.getY() / distanceFromRobotToLine;
        }
        
        // 5) Start building the desiredVelocity by moving towards the line
        ChassisSpeeds desiredVelocity = directionTowardsLine.times(speedTowardsLine);

        // 6) Incorporate the driver's requested speeds along the line,
        //    ignoring any requested speeds that are perpendicular to the line.
        projectionOntoLine = rawSpeedRequest.vxMetersPerSecond * directionVectorAlongLine.getX() + rawSpeedRequest.vyMetersPerSecond * directionVectorAlongLine.getY();
        desiredVelocity.vxMetersPerSecond += projectionOntoLine * directionVectorAlongLine.getX();
        desiredVelocity.vyMetersPerSecond += projectionOntoLine * directionVectorAlongLine.getY();

        // 7) Now rotate the robot so it's facing in the same direction as the line
        this.fieldOrientedDriveWhileAiming(desiredVelocity, lineToDriveOn.getRotation());
    }


    //could be used for a drivetrain command in the future; leave this as its own function
    private void setModuleStates(SwerveModuleState[] desiredStates, boolean closedLoop) {
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


    //**************** ODOMETRY / POSE ESTIMATION ****************/

    /**
     * Sets the current position of the robot on the field in meters.
     * <p>
     * A positive X value brings the robot towards the red alliance,
     * and a positive Y value brings the robot left as viewed by the blue alliance.
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
     * This value considers the origin to be the right side of the blue alliance.
     * <p>
     * A positive X value brings the robot towards the red alliance, and a positive Y value
     * brings the robot towards the left side as viewed from the blue alliance.
     * <p>
     * Rotations are discontinuous counter-clockwise positive, with an angle of 0 facing away from the blue alliance wall.
     * 
     * @return The current position of the robot on the field in meters.
     */ 
    public Pose2d getPoseMeters() {
        return fusedPoseEstimator.getEstimatedPosition();
    }

    /**
     * Gets the rotation reported by the gyro.
     * This rotation is continuous and counterclockwise positive.
     * 
     * This is not necessarily equivalent to the one reported by getPoseMeters(), and it is recommended
     * to use that rotation in almost every case.
     * 
     * This is usable for calibrating the wheel radii, where a continuous angle is required.
     * @return
     */
    public Rotation2d getGyroRotation2d() {
        return gyroInputs.robotYawRotation2d;
    }

    /**
     * Sets the angle of the robot's pose so that it is facing forward, away from your alliance wall. 
     * This allows the driver to realign the drive direction and other calls to our angle.
     */
    public void setRobotFacingForward() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isEmpty()) {
            return;
        }

        Rotation2d newAngle = Rotation2d.fromDegrees(0);
        if (alliance.get() == Alliance.Red) {
            newAngle = Rotation2d.fromDegrees(180);
        }

        Translation2d location = getPoseMeters().getTranslation();

        setPoseMeters(new Pose2d(location, newAngle));
    }


    /**
     * Takes the best estimated pose from the vision, and sets our current poseEstimator pose to this one.
     */
    public void setPoseToVisionMeasurement() {
        if (visionInputs.visionMeasurements.size() > 0)
            setPoseMeters(visionInputs.visionMeasurements.get(0).robotFieldPose);
    }


    private void updatePoseEstimator() {

        fusedPoseEstimator.update(gyroInputs.robotYawRotation2d, getModulePositions());
        wheelsOnlyPoseEstimator.update(gyroInputs.robotYawRotation2d, getModulePositions());

        for (VisionMeasurement visionMeasurement : visionInputs.visionMeasurements) {
            Translation2d visionTranslation = visionMeasurement.robotFieldPose.getTranslation();
            Translation2d estimatedTranslation = fusedPoseEstimator.getEstimatedPosition().getTranslation();

            // don't add vision measurements that are too far away
            // for reference: it is 6 meters from speaker tags to wing.
            double teleportToleranceMeters = 2.0;
            if (visionTranslation.getDistance(estimatedTranslation) <= teleportToleranceMeters) {
                fusedPoseEstimator.addVisionMeasurement(
                    visionMeasurement.robotFieldPose, 
                    visionMeasurement.timestampSeconds, 
                    visionMeasurement.stdDevs
                );
            }
        }
    }

    public boolean inSpeakerShotRange() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isEmpty()) {
            return true; //dummy value, should never happen
        }

        //this position marks a little past the midline (closer to the far alliance)


        if (alliance.get() == Alliance.Blue) {
            return getPoseMeters().getX() <= 7.05; // half way between blue wing and center line
        }

        if (alliance.get() == Alliance.Red) {
            return getPoseMeters().getX() >= 9.49; // half way between red wing and center line
        }

        return true;
    }


    //**************** TARGET TRACKING (Speaker, Note, etc.) ****************/


    public boolean intakeSeesNote() {
        return visionInputs.nearestNoteRobotFrame.isPresent();
    }

    public boolean shouldTrackNote() {
        return intakeSeesNote() && isTrackingNote;
    }

    /**
     * Gets the angle that the front of the robot needs to aim at in order to intake the nearest ring
     * seen on the intake camera. This is used for the rotation override during auto.
     * This rotation2d is empty if you can't see a note.
     */
    public Rotation2d getFieldRelativeRotationToNote() {
        if (visionInputs.nearestNoteRobotFrame.isEmpty())
            return new Rotation2d();

        Rotation2d robotAngle = getPoseMeters().getRotation();
        Rotation2d noteAngleToRobot = visionInputs.nearestNoteRobotFrame.get().toTranslation2d().getAngle().rotateBy(new Rotation2d(Math.PI));

        return robotAngle.plus(noteAngleToRobot);
    }


    public Optional<Rotation2d> getAutoRotationOverride() {
        if (isTrackingSpeakerInAuto) {
            Rotation2d angle = FlyingCircuitUtils.getAngleToFieldElement(FieldElement.SPEAKER, getPoseMeters());
            Logger.recordOutput("PathPlanner/rotationTargetOverride", angle);
            return Optional.of(angle);
        }
        if (isTrackingNote && intakeSeesNote()) {
            Rotation2d angle = getFieldRelativeRotationToNote();
            Logger.recordOutput("PathPlanner/rotationTargetOverride", angle);
            return Optional.empty();//Optional.of(angle);
        }
        else {
            Logger.recordOutput("PathPlanner/rotationTargetOverride", new Rotation2d(0));
            return Optional.empty();
        }
    }

    public void driveTowardsNote() {

        if (visionInputs.nearestNoteRobotFrame.isEmpty()) {
            return;
        }

        double maxAccel = 1.0; // [meters per second per second] (emperically determined)

        double distanceToNote = visionInputs.nearestNoteRobotFrame.get().toTranslation2d().getDistance(new Translation2d());

        // Physics 101: under constant accel -> v_final^2 = v_initial^2 + 2 * accel * displacement
        // displacement = finalDistanceToNote - currentDistanceToNote = 0 - currentDistanceToNote
        // accel = maxAccel
        // v_final = 0 (because we want to come to a controlled stop to pickup the note)
        // after some algebra -> v_initial = sqrt(-2 * accel * displacement)

        double desiredSpeed = Math.sqrt(-2 * maxAccel * (0 - distanceToNote));

        // direction to drive is opposite of the direction to point because the
        // intake is in the back of the robot.
        Rotation2d directionToPoint = this.getFieldRelativeRotationToNote();
        Rotation2d directionToDrive = directionToPoint.rotateBy(Rotation2d.fromDegrees(180));

        ChassisSpeeds desiredVelocity = new ChassisSpeeds();
        desiredVelocity.vxMetersPerSecond = desiredSpeed * directionToDrive.getCos();
        desiredVelocity.vyMetersPerSecond = desiredSpeed * directionToDrive.getSin();

        this.fieldOrientedDriveWhileAiming(desiredVelocity, directionToPoint);
    }


    //**************** MUSIC ****************/

    private void addInstrument(TalonFX kraken) {
        orchestra.addInstrument(kraken);
    }

    public static Orchestra getOrchestra() {
        return orchestra;
    }

    private StatusCode[] playOrchestra() {
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
    private StatusCode stopOrchestra() {
        StatusCode stopCode = orchestra.stop();
        return stopCode;
    }
    public boolean isSongPlaying() {
        return orchestra.isPlaying();
    }
    private int songTimestamp() {
        return ((int)orchestra.getCurrentTime());
    }



    public boolean isAligned() {
        return angleController.atSetpoint();
    }

    public double getAngleError() {
        return angleController.getPositionError();
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

        Logger.recordOutput(
            "drivetrain/swerveModulePositions", 
            new SwerveModulePosition[] {
                swerveModules[0].getPosition(),
                swerveModules[1].getPosition(),
                swerveModules[2].getPosition(),
                swerveModules[3].getPosition()
            });

        Logger.recordOutput("drivetrain/anglePIDSetpoint", Rotation2d.fromDegrees(angleController.getSetpoint()));
        Logger.recordOutput("drivetrain/isAligned", isAligned());
        Logger.recordOutput("drivetrain/isTrackingNote", isTrackingNote);
        Logger.recordOutput("drivetrain/fieldRelativeRotationToNote", getFieldRelativeRotationToNote());


        // Note tracking visualization
        if (visionInputs.nearestNoteRobotFrame.isPresent()) {
            Translation2d noteRelativeToRobot = visionInputs.nearestNoteRobotFrame.get().toTranslation2d();
            Pose2d noteRelativeToField = getPoseMeters().plus(new Transform2d(noteRelativeToRobot, new Rotation2d()));
            Logger.recordOutput("drivetrain/trackedNotePose", noteRelativeToField);
        }
        else {
            Logger.recordOutput("drivetrain/trackedNotePose", getPoseMeters());
        }
    }
}
