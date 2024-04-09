// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;
import java.util.Arrays;

import com.pathplanner.lib.path.PathConstraints;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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
    public final static class ShooterConstants {
        /**Rotations of the motor per rotations of the wheel; a number greater than 1 represents a reduction. */
        public final static double flywheelGearReduction = 1.;

        public static final double flywheelCircumferenceMeters = Units.inchesToMeters(4)*Math.PI;
    
        public final static double kPFlywheelsVoltsSecondsPerMeter = .6;
        public final static double kIFlywheelsVoltsPerMeter = 0.;
        public final static double kDFlywheelsVoltsSecondsSquaredPerMeter = 0.;

        public final static double kSFlywheelsVolts = 0.0;
        public final static double kVFlywheelsVoltsSecondsPerMeter = 0.4185;
        public final static double kAFlywheelsVoltsSecondsSquaredPerMeter = 0.;

        public final static int leftMotorID = 6;
        public final static int rightMotorID = 5;

        public final static int indexerMotorID = 4;

        public static final int indexerProximitySwitchIDLeft = 3;
        public static final int indexerProximitySwitchIDRight = 1;

        public static final double kPIndexerVoltsPerRPS = 0.04;

        public static final double kSIndexerVolts = 0;
        public static final double kVIndexerVoltsPerRPS = 0.130;

        /** Rotations of the black roller of the indexer per rotation of the motor. */
        public static final double indexerBlackRollerGearRatio = (17.0 / 37.0) * (18.0 / 24.0);
        public static final double blackRollerRadiusMeters = Units.inchesToMeters(2.125/2.0);
        public static final double blackRollerCircumferenceMeters = 2 * Math.PI * blackRollerRadiusMeters;

        /** Rotations of the orange wheels on the indexer per rotation of the motor. */
        public static final double indexerOrangeWheelsGearRatio = (17.0 / 37.0);
        public static final double orangeWheelsRadiusMeters = Units.inchesToMeters(1.125);
        public static final double orangeWheelsCircumferenceMeters = 2 * Math.PI * orangeWheelsRadiusMeters;

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
        public static final double drivetrainRadiusMeters = Math.hypot(wheelbaseMeters / 2.0, trackwidthMeters / 2.0); //0.4177


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
            SwerveModuleConstants.driveGearReduction * SwerveModuleConstants.wheelCircumferenceMeters; // ~5.06 m/s
                                                                                                       // actual top speed based on characterization is ~4.46 m/s (12 volts / 2.69 (volts / [meter / second]))
                                                                                                       // top speed that seems controllable in auto is ~4.0 m/s

        /**
         * This is the max desired speed that will be achievable in teleop.
         * <br>
         * If the controller joystick is maxed in one direction, it will drive at this speed.
         * <br>
         * This value will be less than or equal to the maxAchievableVelocityMetersPerSecond, depending on driver preference.
         */
        public static final double maxDesiredTeleopVelocityMetersPerSecond = maxAchievableVelocityMetersPerSecond; 

        /**
         * The maximum achievable angular velocity of the robot in radians per second.
         * <br>
         * This is a measure of how fast the robot can rotate in place, based off of maxAchievableVelocityMetersPerSecond.
         */
        public static final double maxAchievableAngularVelocityRadiansPerSecond = maxAchievableVelocityMetersPerSecond / drivetrainRadiusMeters; // Theoretical ~1.93 rotations per second
                                                                                                                                                 // using 4.46 m/s for max linear speed yeilds ~1.7 rotations per second
                                                                                                                                                 // using 4.0 m/s for max linear speed yeilds 1.52 rotations per second
                                                                                                                                                 // we use 1.0 rotations per second in auto to be extra conservative
                                                                                                                                                 // because any time you're rotating, you're taking away from your translational speed.

        /**
         * This is the max desired angular velocity that will be achievable in teleop.
         * <br>
         * If the controller rotation joystick is maxed in one direction, it will rotate at this speed.
         * <br>
         * This value will be tuned based off of driver preference.
         */
        public static final double maxDesiredTeleopAngularVelocityRadiansPerSecond = Units.rotationsToRadians(0.85);


        public static final PathConstraints pathfindingConstraints = new PathConstraints(
                1.0, 1.0,
                Units.degreesToRadians(360), Units.degreesToRadians(360));
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

        // The wheels have a 2 inch radius, but sink into the capet about (1/16) of an inch.
        // As an estimate, the wheel radius is Units.inchesToMeters(2.-1./16.), or 0.0492m
        public static final double wheelRadiusMeters = 0.0496; //use MeasureWheelDiameter for this!
        public static final double wheelCircumferenceMeters = 2 * Math.PI * wheelRadiusMeters; // ~0.31

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
        public static final double drivekVVoltsSecondsPerMeter = 2.42;
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
        // original value 137, changed to 129 for inspection.
        // As of 4/3/2024, now has to be 122 due to new black banebot wheels for hardstop
        public final static double armMaxAngleDegrees = 122.0;  

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
        public final static double kDArmVoltsSecondsPerDegree = 0.01;

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

        //abbreviations so practiceFieldTags isn't heinously long
        private static double im(double inches) {
            return Units.inchesToMeters(inches);
        }
        private static double dr(double degrees) {
            return Units.degreesToRadians(degrees);
        }

        //tags for practice field: these are identical to the real field,
        //except the two speaker tags are aligned with the red alliance wall
        //rather than 1.5 inches into the wall.

        //these only include the red side of the field, since our practice field only uses those tags.
        //the pose still follows the default field coordinate system, with a blue origin
        private final static List<AprilTag> practiceFieldTags = Arrays.asList(
            //source tag
            new AprilTag(1, new Pose3d(
                new Translation3d(im(593.68), im(9.68), im(53.38)),
                new Rotation3d(0, 0, dr(120)))),
            //source tag
            new AprilTag(2, new Pose3d(
                new Translation3d(im(637.21), im(34.79), im(53.38)),
                new Rotation3d(0, 0, dr(120)))),
            //speaker tag (shifted relative to real field position)
            new AprilTag(3, new Pose3d(
                new Translation3d(im(652.73-1.5), im(196.17), im(57.13)),
                new Rotation3d(0, 0, dr(180)))),
            //speaker tag (shifted relative to field field position)
            new AprilTag(4, new Pose3d(
                new Translation3d(im(652.73-1.5), im(218.42), im(57.13)),
                new Rotation3d(0, 0, dr(180)))),
            //amp tag
            new AprilTag(5, new Pose3d(
                new Translation3d(im(578.77), im(323.00), im(53.38)),
                new Rotation3d(0, 0, dr(270)))),
            //stage left
            new AprilTag(11, new Pose3d(
                new Translation3d(im(468.69), im(146.19), im(52.00)),
                new Rotation3d(0, 0, dr(300)))),
            //stage right
            new AprilTag(12, new Pose3d(
                new Translation3d(im(468.69), im(177.10), im(52.00)),
                new Rotation3d(0, 0, dr(60)))),
            new AprilTag(13, new Pose3d(
                new Translation3d(im(441.74), im(161.62), im(52.00)),
                new Rotation3d(0, 0, dr(180))))
        );
    

        //for use on real field:
        //public final static AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
        
        //for use on practice field:
        public final static AprilTagFieldLayout aprilTagFieldLayout = new AprilTagFieldLayout(
            practiceFieldTags, 16, 8.2
        );
    
        public final static Transform3d robotToShooterCamera = new Transform3d(
            new Translation3d(Units.inchesToMeters(10.25), 0, Units.inchesToMeters(9.5)), // 11.5 inches off the ground, and 8 inches forward from the center of the robot
            new Rotation3d(0, Math.toRadians(-22), 0)
        );

        public final static Transform3d robotToTrapCamera = new Transform3d(
            new Translation3d(Units.inchesToMeters(-11), 0, Units.inchesToMeters(11.5)),
            new Rotation3d(0, Math.toRadians(-33), Math.toRadians(180))
        );

        public final static Transform3d robotToNoteCamera = new Transform3d(
            new Translation3d(Units.inchesToMeters(-11.25), 0, Units.inchesToMeters(15.25)),
            new Rotation3d(0, Math.toRadians(24), Math.toRadians(180))
        );
    }

    public static enum FieldElement {
        SPEAKER, AMP, STAGE_LEFT, STAGE_RIGHT, CENTER_STAGE, LOB_TARGET, CARPET,
        NOTE_1, NOTE_2, NOTE_3, NOTE_4, NOTE_5, NOTE_6, NOTE_7, NOTE_8
    }



    public final static class FieldConstants {

        /** Distance from the front edge of the speaker structure to the carpet. */
        public final static double speakerLowerEdgeHeightMeters = Units.inchesToMeters((6*12)+6);
        public final static double speakerUpperEdgeHeightMeters = Units.inchesToMeters((6*12) + 10 + (7.0/8.0));
        public final static double actualSpeakerHeightMeters = (speakerLowerEdgeHeightMeters + speakerUpperEdgeHeightMeters) / 2.0;

        /** Our shot was always landing a few inches too high, so this fudge factor was added
         *  and seemed to do the trick. If I had to guess, I'd say that this effectively
         *  accounts for the difference in height between the pivot point of the arm
         *  and the actual height of the note when it exits the shooter.
         *  The note will be a few inches higher because the shooter is aimed upward.
         *  It could also have something to do with the note exiting the shooter
         *  closer to the front of the robot than the pivot point is?
         *  Either way, it works for now, and we'll probably just stick with the fudge factor
         *  in the interest of time.
         */
        public final static double speakerHeightFudgeFactorMeters = Units.inchesToMeters(-3.5*0); // accounts for difference in height between actuall note exit point and the pivot point. exit point is above the pivot, so effective vertical distance is less
        public final static double speakerHeightMeters = actualSpeakerHeightMeters + speakerHeightFudgeFactorMeters;

        /** X-Y position of the april tag at the center of the red speaker.  
         *  Copied from https://firstfrc.blob.core.windows.net/frc2024/FieldAssets/2024LayoutMarkingDiagram.pdf
         */
        public final static Translation2d blueSpeakerTranslation2d = new Translation2d(Units.inchesToMeters(-1.5), Units.inchesToMeters(218.42));

        /** X-Y position of the april tag at center of the blue speaker.
         *  Copied from https://firstfrc.blob.core.windows.net/frc2024/FieldAssets/2024LayoutMarkingDiagram.pdf
         */
        public final static Translation2d redSpeakerTranslation2d = new Translation2d(Units.inchesToMeters(652.73), Units.inchesToMeters(218.42));

        /** Distance from the floor to the center of the pivot. This is used for angle calculations for shoot from anywhere. */
        public final static double pivotHeightMeters = Units.inchesToMeters(23.5);
        
        /** Horizontal distance from the robot center to the pivot center */
        public final static double pivotOffsetMeters = Units.inchesToMeters(9); // 22 centimeters
    }

    public final static class IntakeConstants {
        public final static int frontIntakeMotorID = 4;
        public final static int backIntakeMotorID = 3;
        public static final int intakeProximitySwitchIDLeft = 2;
        public static final int intakeProximitySwitchIDRight = 0;

        

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

        /** TODO: documentation */
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
