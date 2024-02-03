
package frc.robot.subsystems.drivetrain;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.SwerveModuleConstants;

public class Drivetrain extends SubsystemBase {

    private GyroIO gyroIO;
    private GyroIOInputsAutoLogged gyroInputs;

    private SwerveModule[] swerveModules;

    private SlewRateLimiter chassisSpeedsXSlewLimiter;
    private SlewRateLimiter chassisSpeedsYSlewLimiter;

    private SwerveDriveOdometry swerveOdometry;

    private Pose2d poseMeters;

    public Drivetrain(
        GyroIO gyroIO, 
        SwerveModuleIO flSwerveModuleIO, 
        SwerveModuleIO frSwerveModuleIO, 
        SwerveModuleIO blSwerveModuleIO, 
        SwerveModuleIO brSwerveModuleIO
    ) {

        this.gyroIO = gyroIO;
        gyroInputs = new GyroIOInputsAutoLogged();

        swerveModules = new SwerveModule[] {
            new SwerveModule(flSwerveModuleIO, 0),
            new SwerveModule(frSwerveModuleIO, 1),
            new SwerveModule(blSwerveModuleIO, 2),
            new SwerveModule(brSwerveModuleIO, 3)
        };

        gyroIO.setRobotYaw(0);
        swerveOdometry = new SwerveDriveOdometry(DrivetrainConstants.swerveKinematics, gyroInputs.robotYawRotation2d, getModulePositions());

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
        swerveOdometry.resetPosition(gyroInputs.robotYawRotation2d, getModulePositions(), pose);
        poseMeters = pose;
    }

    /**
     * Gets the current position of the robot on the field in meters.
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

    @Override
    public void periodic() {
        gyroIO.updateInputs(gyroInputs);
        for (SwerveModule mod : swerveModules)
            mod.periodic();
        Logger.processInputs("gyroInputs", gyroInputs);


        poseMeters = swerveOdometry.update(gyroInputs.robotYawRotation2d, getModulePositions());
        


        Logger.recordOutput("drivetrain/swerveOdometry", getPoseMeters());

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
