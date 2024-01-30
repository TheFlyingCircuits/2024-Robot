
package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.SwerveModuleConstants;

public class Drivetrain extends SubsystemBase {

    private GyroIO gyroIO;
    private GyroIOInputsAutoLogged gyroInputs;

    private SwerveModule[] swerveModules;

    private SlewRateLimiter chassisSpeedsXSlewLimiter;
    private SlewRateLimiter chassisSpeedsYSlewLimiter;

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

        chassisSpeedsXSlewLimiter = new SlewRateLimiter(DrivetrainConstants.maxDesiredTeleopAccelMetersPerSecondSquared);
        chassisSpeedsYSlewLimiter = new SlewRateLimiter(DrivetrainConstants.maxDesiredTeleopAccelMetersPerSecondSquared);
    }

    /**
   * Sets the gyroscope angle to zero.
   * <br>
   * This can be used to set the direction the robot is currently facing to the 'forwards' direction.
   */
    public void setYaw(double degrees) {
        gyroIO.setRobotYaw(degrees);
        //TODO: reset odometry when zeroing yaw
    }

    /**
   * Gets the angle of the robot measured by the gyroscope as a Rotation2d (continuous).
   * @return rotation2d - this angle will be counterclockwise positive.
   */
    public Rotation2d getRobotRotation2d() {
        return Rotation2d.fromDegrees(gyroInputs.robotYawDegrees);
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

    @Override
    public void periodic() {
        for (SwerveModule mod : swerveModules) {
            mod.periodic();
        }
        gyroIO.updateInputs(gyroInputs);


    }
}
