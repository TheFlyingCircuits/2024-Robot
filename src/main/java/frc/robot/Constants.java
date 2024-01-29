// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public final class ShooterConstants {
        /**Rotations of the wheel per rotations of the motor; a number greater than 1 represents a reduction. */
        public final static double kFlywheelGearReduction = 1.;
    
        public final static double kPFlywheelsVoltsSecondsPerRotation = 0.;
        public final static double kIFlywheelsVoltsPerRotation = 0.;
        public final static double kDFlywheelsVoltsSecondsSquaredPerRotation = 0.;

        public final static double kSFlywheelsVolts = 0.;
        public final static double kVFlywheelsVoltsSecondsPerRotation = 0.;
        public final static double kAFlywheelsVoltsSecondsSquaredPerRotation = 0.;
    }

    public static final class DrivetrainConstants {
        // KINEMATICS CONSTANTS
        /**
         * Distance between the center point of the left wheels and the center point of the right wheels.
         */
        public static final double trackwidthMeters = Units.inchesToMeters(22.75);
        /**
         * Distance between the center point of the front wheels and the center point of the back wheels.
         */
        public static final double wheelbaseMeters = Units.inchesToMeters(22.75);


        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelbaseMeters / 2.0, trackwidthMeters / 2.0),
            new Translation2d(wheelbaseMeters / 2.0, -trackwidthMeters / 2.0),
            new Translation2d(-wheelbaseMeters / 2.0, trackwidthMeters / 2.0),
            new Translation2d(-wheelbaseMeters / 2.0, -trackwidthMeters / 2.0));




        /**
         * The maximum possible velocity of the robot in meters per second.
         * <br>
         * This is a measure of how fast the robot will be able to drive in a straight line, based off of the empirical free speed of the drive NEOs.
         */
        public static final double maxAchievableVelocityMetersPerSecond = 5880.0 / 60.0 *
            SwerveModuleConstants.driveGearReduction *
            SwerveModuleConstants.wheelDiamaterMeters * Math.PI;

        /**
         * This is the max desired speed that will be achievable in teleop.
         * <br>
         * If the controller joystick is maxed in one direction, it will drive at this speed.
         * <br>
         * This value will be less than or equal to the maxAchievableVelocityMetersPerSecond, depending on driver preference.
         */
        public static final double maxDesiredTeleopVelocityMetersPerSecond = 4.3;

        /**
         * The maximum achievable angular velocity of the robot in radians per second.
         * <br>
         * This is a measure of how fast the robot can rotate in place, based off of maxAchievableVelocityMetersPerSecond.
         */
        public static final double maxAchievableAngularVelocityRadiansPerSecond = maxAchievableVelocityMetersPerSecond /
            Math.hypot(trackwidthMeters / 2.0, wheelbaseMeters / 2.0);

        /**
         * This is the max desired angular velocity that will be achievable in teleop.
         * <br>
         * If the controller rotation joystick is maxed in one direction, it will rotate at this speed.
         * <br>
         * This value will be tuned based off of driver preference.
         */
        public static final double maxDesiredTeleopAngularVelocityRadiansPerSecond = 5.4;


        public static final double maxDesiredTeleopAccelMetersPerSecondSquared = 27.27;


    }

    public final class SwerveModuleConstants {
        /** Rotations of the drive wheel per rotations of the drive motor. */
        public static final double driveGearReduction = (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0);

        /** Rotations of the steering column per rotations of the angle motor. */
        public static final double steerGearReduction = (14.0 / 50.0) * (10.0 / 60.0);

        public static final double wheelDiamaterMeters = 0.10033;
        public static final double wheelCircumferenceMeters = wheelDiamaterMeters * Math.PI;

        // PID + FEEDFORWARD CONSTANTS FOR MOTORS
        // PID for drive motors.
        public static final double drivekPVoltsPerMeterPerSecond = 0.81027; //sysid says this should 0.27
        public static final double drivekIVoltsPerMeter = 0.;
        public static final double drivekDVoltsPerMeterPerSecondSquared = 0.00;

        // PID for angle motors.
        public static final double anglekPVoltsPerDegree = 0.08;//0.065;
        public static final double anglekIVoltsPerDegreeSeconds = 0.; // this might be the wrong unit idk 
        public static final double anglekDVoltsPerDegreePerSecond = 0.;

        public static final double drivekSVolts = 0.1301;
        public static final double drivekVVoltsSecondsPerMeter = 2.6931; // .8679
        public static final double drivekAVoltsSecondsSquaredPerMeter = 0.43963;

        // Motor configs
        public static final int angleContinuousCurrentLimit = 50;
        public static final boolean angleInvert = true;
        public static final IdleMode angleNeutralMode = IdleMode.kBrake;
        
        public static final int driveContinuousCurrentLimit = 60;
        public static final boolean driveInvert = true;
        public static final IdleMode driveNeutralMode = IdleMode.kBrake;

    }

    public final class ArmConstants {
        /**Rotations of the arm per rotations of the motor; a number greater than 1 represents a reduction. */
        public final static double kArmGearReduction = 1.;

        /**Minimum angle of the arm, in degrees. This value should be negative, as it is below the horizontal.*/
        public final static double kArmMinAngleDegrees = -50.;

        /**Maximum angle of the arm, in degrees. This value should be positive and greater than 90, as it is beyond the vertical. */
        public final static double kArmMaxAngleDegrees = 100.;
    }
}
