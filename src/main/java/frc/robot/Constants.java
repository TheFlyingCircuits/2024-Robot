// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public final static class ShooterConstants {
        /**Rotations of the motor per rotations of the wheel; a number greater than 1 represents a reduction. */
        public final static double flywheelGearReduction = 1.;

        public static final double flywheelCircumferenceMeters = Units.inchesToMeters(3.875)*Math.PI;
    
        public final static double kPFlywheelsVoltsSecondsPerMeter = .2;
        public final static double kIFlywheelsVoltsPerMeter = 0.;
        public final static double kDFlywheelsVoltsSecondsSquaredPerMeter = 0.;

        public final static double kSFlywheelsVolts = 0.0;
        public final static double kVFlywheelsVoltsSecondsPerMeter = 0.4185;
        public final static double kAFlywheelsVoltsSecondsSquaredPerMeter = 0.;

        public final static int leftMotorID = 6;
        public final static int rightMotorID = 5;

        public final static int indexerMotorID = 4;

        public static final int indexerProximitySwitchID = 1;

        public static final double kPIndexerVoltsPerRPM = 0.0001;

        public static final double kSIndexerVolts = 0;
        public static final double kVIndexerVoltsPerRPM = 0.0031;

        /** Rotations of the motor per rotations of the indexer wheel; a number greater than 1 represents a reduction. */
        public static final double indexerGearReduction = 24./18.;

        public static final double motorMaxTempCelsius = 70;
    }

    public final static class DrivetrainConstants {
        // KINEMATICS CONSTANTS

        /**
         * Distance between the center point of the left wheels and the center point of the right wheels.
         */
        public static final double trackwidthMeters = Units.inchesToMeters(23.75);
        /**
         * Distance between the center point of the front wheels and the center point of the back wheels.
         */
        public static final double wheelbaseMeters = Units.inchesToMeters(22.75);
        /**
         * Distance from the center of the robot to each swerve module.
         */
        public static final double drivetrainRadiusMeters = Math.hypot(wheelbaseMeters / 2.0, trackwidthMeters / 2.0);


        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelbaseMeters / 2.0, trackwidthMeters / 2.0),
            new Translation2d(wheelbaseMeters / 2.0, -trackwidthMeters / 2.0),
            new Translation2d(-wheelbaseMeters / 2.0, trackwidthMeters / 2.0),
            new Translation2d(-wheelbaseMeters / 2.0, -trackwidthMeters / 2.0));




        /**
         * The maximum possible velocity of the robot in meters per second.
         * <br>
         * This is a measure of how fast the robot will be able to drive in a straight line, based off of the empirical free speed of the drive Krakens.
         */
        public static final double krakenFreeSpeedRPM = 5800;
        public static final double krakenFreeSpeedRotationsPerSecond = krakenFreeSpeedRPM / 60.;
        public static final double maxAchievableVelocityMetersPerSecond = krakenFreeSpeedRotationsPerSecond *
            SwerveModuleConstants.driveGearReduction * SwerveModuleConstants.wheelCircumferenceMeters;

        /**
         * This is the max desired speed that will be achievable in teleop.
         * <br>
         * If the controller joystick is maxed in one direction, it will drive at this speed.
         * <br>
         * This value will be less than or equal to the maxAchievableVelocityMetersPerSecond, depending on driver preference.
         */
        public static final double maxDesiredTeleopVelocityMetersPerSecond = maxAchievableVelocityMetersPerSecond; // TODO: placeholder

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


        public static final double maxDesiredTeleopAccelMetersPerSecondSquared = 100.0; // TODO: placeholder


    }

    public final static class ControllerConstants {
        public static final double controllerDeadzone = 0.175;
        public static final double maxThrottle = 1.0;
    }

    public final static class MotorConstants {
        public static final int universalCurrentLimitAmps = 50;

        // Motor configs
        public static final int angleContinuousCurrentLimit = 50;
        public static final boolean angleInvert = true;
        public static final IdleMode angleNeutralMode = IdleMode.kBrake;
        
        public static final int driveContinuousCurrentLimit = 60;
        public static final boolean driveInvert = true;
        public static final IdleMode driveNeutralMode = IdleMode.kBrake;
    }

    public final static class SwerveModuleConstants {
        /** Rotations of the drive wheel per rotations of the drive motor. */
        public static final double driveGearReduction = (16.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0);

        /** Rotations of the steering column per rotations of the angle motor. */
        public static final double steerGearReduction = (14.0 / 50.0) * (10.0 / 60.0);

        // The wheels have a 2 inch radius, but sink into the capet about (1/8) of an inch
        // for an effective radius of 2-(1/8).
        public static final double wheelRadiusMeters = Units.inchesToMeters(2.-1./8.);
        public static final double wheelCircumferenceMeters = 2 * Math.PI * wheelRadiusMeters;

        // PID + FEEDFORWARD CONSTANTS FOR MOTORS
        // PID for drive motors.
        public static final double drivekPVoltsPerMeterPerSecond = 0;
        public static final double drivekIVoltsPerMeter = 0.;
        public static final double drivekDVoltsPerMeterPerSecondSquared = 0.;

        // PID for angle motors.
        public static final double anglekPVoltsPerDegree = 0.08;
        public static final double anglekIVoltsPerDegreeSeconds = 0.; // this might be the wrong unit idk 
        public static final double anglekDVoltsPerDegreePerSecond = 0.;

        public static final double drivekSVolts = 0.;
        public static final double drivekVVoltsSecondsPerMeter = 2.69;//12.0/DrivetrainConstants.maxAchievableVelocityMetersPerSecond; // TODO: this is a placeholder
        public static final double drivekAVoltsSecondsSquaredPerMeter = 0.;

    }

    public final static class GyroConstants {
        public static final int pigeonID = 0;


        //Follow the mount calibration process in Phoenix Tuner to determine these
        public static final double mountPoseYawDegrees = 0.7333114147186279;
        public static final double mountPosePitchDegrees = -0.11852765083312988;
        public static final double mountPoseRollDegrees = -1.0425487756729126;
    }

    public final static class ArmConstants {
        /**Rotations of the motor per rotations of the arm; a number greater than 1 represents a reduction. */
        public final static double armGearReduction = 460./3.;

        /**Minimum angle of the arm, in degrees. This value should be negative, as it is below the horizontal.*/
        public final static double armMinAngleDegrees = -25.8;

        /**Maximum angle of the arm, in degrees. This value should be positive and greater than 90, as it is beyond the vertical. */
        public final static double armMaxAngleDegrees = 143.;

        public final static double armMaxVelDegreesPerSecond = 360.;

        public final static double armMaxAccelDegreesPerSecondSquared = 660.;

        /**temporary ids for motors and CANcoder of the shooting arm */
        public final static int leftMotorID = 7;
        public final static int rightMotorID = 8;
        public final static int leftArmCANcoderID = 9;
        public final static int rightArmCANcoderID = 8;
        

        public final static double rightArmCANcoderOffset = -0.431;
        public final static double leftArmCANcoderOffset = 0.100;

        /***** REAL CONSTANTS ******/
        public final static double kSArmVolts = 0.005;
        public final static double kGArmVolts = 0.32;
        public final static double kVArmVoltsSecondsPerRadian = 3.1;
        public final static double kAArmVoltsSecondsSquaredPerRadian = 0;

        public final static double kPArmVoltsPerDegree = 0.3;
        public final static double kDArmVoltsSecondsPerDegree = 0.005;

        /**** SIM CONSTANTS  ******/
        // public final static double kSArmVolts = 0.0;
        // public final static double kGArmVolts = 0.30;
        // public final static double kVArmVoltsSecondsPerRadian = 3.5;
        // public final static double kAArmVoltsSecondsSquaredPerRadian = 0;
        // public final static double kPArmVoltsPerDegree = 0.3;
        // public final static double kIArmVoltsPerDegreesSeconds = 0.;
        // public final static double kDArmVoltsSecondsPerDegree = 0.1;

        public final static TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(
            armMaxVelDegreesPerSecond, armMaxAccelDegreesPerSecondSquared
        );
    }

    public final static class ClimbConstants {
        
        /**
         * temp motor ids for climbing motors
         */
        public final static int leftMotorID = 9;
        public final static int rightMotorID = 10;

        /**
         * Use this value as the conversion factor between the motors rotations and meters that the climb arms have extended.
         * A positive rotation will result in a positive extension.
         * This is plugged directly into the climb encoders' setPositionConversionFactor method.
         */
        public final static double climberArmMetersPerMotorRotation = Units.inchesToMeters(2.256*Math.PI/21.);

        public final static double climbMaxPosMeters = .53;
        public final static double climbMinPosMeters = -0.14;

    }

    public final static class VisionConstants {

        public final static AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    
        public final static Transform3d robotToCamera = new Transform3d(
            new Translation3d(Units.inchesToMeters(8), 0, Units.inchesToMeters(11.5)),
            new Rotation3d(0, Math.toRadians(-28), 0)
        );
    }

    public final static class FieldConstants {

        /** Distance from the front edge of the speaker structure to the carpet. */
        public final static double speakerHeightMeters = 2.09;

        /** X-Y position of the center of 30 cm behind the front edge of the red speaker.  */
        public final static Translation2d blueSpeakerTranslation2d = new Translation2d(0.46-0.3, 5.55);

        /** X-Y position of the center of 30 cm behind the front edge of the blue speaker. */
        public final static Translation2d redSpeakerTranslation2d = new Translation2d(16.08+0.3, 5.55);

        /** Distance from the floor to the center of the pivot. This is used for angle calculations for shoot from anywhere. */
        public final static double pivotHeightMeters = Units.inchesToMeters(22);
        
        /** Horizontal distance from the robot center to the pivot center */
        public final static double pivotOffsetMeters = 0.22; // 22 centimeters
    }

    public final static class IntakeConstants {
        public final static int frontIntakeMotorID = 4;
        public final static int backIntakeMotorID = 3;
        public static final int intakeProximitySwitchID = 0;

        

        public static final double kSFrontIntakeVolts = 0;
        public static final double kVFrontIntakeVoltsPerRPM = 0;
        
        public static final double kSBackIntakeVolts = 0;
        public static final double kVBackIntakeVoltsPerRPM = 0;

        public static final double kPBackIntakeVoltsPerRPM = 0;
        public static final double kPFrontIntakeVoltsPerRPM = 0;
    }

    public final static class LEDConstants {
        public final static int ledPWMPort = 0;

        //total number of leds
        public final static int ledsPerStrip = 60;
        

        public final static double stripLengthMeters = 1.0;

        public final static double ledsPerMeter = (1.0 * ledsPerStrip) / stripLengthMeters;

        public final static double metersPerLed = 1/ledsPerMeter;

        public final static double topThirdBreakpoint = stripLengthMeters - 0.24;
        public final static double bottomThirdBreakpoint = 0.29;

        /**
         * Hues for specific colors
         * Values use the openCV convention where hue ranges from [0, 180)
         */
        public final static class Hues {

            public final static int orangeSignalLight = 4;
            public final static int blueBumpers = 114;
            public final static int redBumpers = 0;
            public final static int redTrafficLight = 0;//0;
            public final static int greenTrafficLight = 40;//60;
            public final static int betweenBlueAndRed = 150; // a purple/pink that's between blue and red.

        }
    }

}
