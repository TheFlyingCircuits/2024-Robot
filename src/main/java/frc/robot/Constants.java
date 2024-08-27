// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;
import java.util.Optional;
import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.Arrays;

import com.pathplanner.lib.path.PathConstraints;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public final static boolean atCompetition = false;
    public final static boolean isDemoMode = true;

    public final static class ShooterConstants {
        /**Rotations of the motor per rotations of the wheel; a number greater than 1 represents a reduction. */
        public final static double flywheelGearReduction = 1.;
        public final static double flywheelCircumferenceMeters = Units.inchesToMeters(4)*Math.PI;
    
        public final static double kPFlywheelsVoltsSecondsPerMeter = .6;
        public final static double kIFlywheelsVoltsPerMeter = 0.;
        public final static double kDFlywheelsVoltsSecondsSquaredPerMeter = 0.;

        public final static double kSFlywheelsVolts = 0.2735;
        public final static double kVFlywheelsVoltsSecondsPerMeter = 0.4047;
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
            new Translation2d(-wheelbaseMeters / 2.0, -trackwidthMeters / 2.0)
        );




        /**
         * The maximum possible velocity of the robot in meters per second.
         * <br>
         * This is a measure of how fast the robot will be able to drive in a straight line, based off of the empirical free speed of the drive Krakens.
         */
        public static final double krakenFreeSpeedRPM = 5800;
        public static final double krakenFreeSpeedRotationsPerSecond = krakenFreeSpeedRPM / 60.;
        public static final double maxAchievableVelocityMetersPerSecond = krakenFreeSpeedRotationsPerSecond *
            SwerveModuleConstants.driveGearReduction * SwerveModuleConstants.wheelCircumferenceMeters; // ~5.23 using a theoretical wheel radius of 2 inches m/s
                                                                                                       // ~5.06 when adding 1/16 of an inch of wheel sink into the carpet.
                                                                                                       // ~5.10 using an emperical measurement of wheel radius on fresh wheels.
                                                                                                       // Actual top speed based on testing is ~4.7 m/s
                                                                                                       // (calculating top speed using kv yeilds [12 / 2.42] ~ 4.96 m/s,
                                                                                                       //  but I don't think we can actually achieve this because 
                                                                                                       //  the battery voltage will likely drop below 12 when all drive motors are running)
                                                                                                       // To give ourselves a little breathing room, we use a max speed of 4.5 m/s in auto.

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
                                                                                                                                                 // using 4.7 m/s for max linear speed yeilds ~1.79 rotations per second
                                                                                                                                                 // using 4.5 m/s for max linear speed yeilds ~1.71 rotations per second
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
        public static final double wheelRadiusMeters = 0.04946; //use MeasureWheelDiameter for this!
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

        public static final double drivekSVolts = 0.2383;
        public static final double drivekVVoltsSecondsPerMeter = 2.2859; // TODO: add desmos link
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
        // As of 4/3/2024, now has to be 123 due to new black banebot wheels for hardstop
        public final static double armMaxAngleDegrees = 123.0;  

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

        /** Distance from the floor to the center of the pivot. This is used for angle calculations for shoot from anywhere. */
        public final static double pivotHeightMeters = Units.inchesToMeters(23.5);
        
        /** Horizontal distance from the robot center to the pivot center */
        public final static double pivotOffsetMeters = Units.inchesToMeters(9); // 22 centimeters

        public final static double armLengthMeters = Units.inchesToMeters(18.25); // 18.29 something from cad?

        public final static double momentOfInertiaKgMSquared = 0.8;
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

        public static AprilTagFieldLayout getPracticeFieldTagLayout() {
            AprilTagFieldLayout officialLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

            List<AprilTag> practiceFieldTags = new ArrayList<AprilTag>(officialLayout.getTags());

            for (AprilTag tag : practiceFieldTags) {
                // Our red speaker tags are in line with the red alliance wall,
                // but they are recessed 1.5 inches behind the wall on an official field.
                if (tag.ID == 3 || tag.ID == 4) {
                    Translation3d officialLocation = tag.pose.getTranslation();
                    double x = officialLocation.getX() - Units.inchesToMeters(1.5);
                    double y = officialLocation.getY();
                    double z = officialLocation.getZ();
                    tag.pose = new Pose3d(new Translation3d(x, y, z), tag.pose.getRotation());
                }

                // set the tag on the pit to be the red amp when in the basement for temporary testing.
                boolean inBasement = false;
                if (inBasement && tag.ID == 12) {
                    tag.pose = officialLayout.getTagPose(5).get();
                }
            }

            return new AprilTagFieldLayout(practiceFieldTags, officialLayout.getFieldLength(), officialLayout.getFieldWidth());
        }
    

        public final static AprilTagFieldLayout aprilTagFieldLayout = atCompetition ? AprilTagFields.k2024Crescendo.loadAprilTagLayoutField()
                                                                      : getPracticeFieldTagLayout();
                                                                      
        public final static Transform3d robotToShooterCamera = new Transform3d(
            new Translation3d(Units.inchesToMeters(9.25), 0, Units.inchesToMeters(10.625)),
            new Rotation3d(0, Math.toRadians(-28), 0)
        );

        public final static Transform3d robotToTrapCamera = new Transform3d(
            new Translation3d(Units.inchesToMeters(-9.625), 0, Units.inchesToMeters(10.63)),
            new Rotation3d(0, Math.toRadians(-32), Math.toRadians(180))
        );

        public final static Transform3d robotToLeftCamera = new Transform3d(
            new Translation3d(Units.inchesToMeters(-5.438), Units.inchesToMeters(12.375), Units.inchesToMeters((16.875))),
            new Rotation3d(0, Math.toRadians(-27), Math.toRadians(90))
        );
        
        public final static Transform3d robotToRightCamera = new Transform3d(
            new Translation3d(Units.inchesToMeters(-5.438), Units.inchesToMeters(-12), Units.inchesToMeters(16.025)),
            new Rotation3d(0, Math.toRadians(-27), Math.toRadians(-90))
        );

        public final static String[] cameraNames = {
            "shooterCamera",
            "trapCamera",
            "leftCamera",
            "rightCamera"
        };

        public final static Transform3d tagCameraTransforms[] = {
            robotToShooterCamera,
            robotToTrapCamera,
            robotToLeftCamera,
            robotToRightCamera
        };

        public final static Transform3d robotToNoteCamera = new Transform3d(
            new Translation3d(Units.inchesToMeters(-10.437), 0, Units.inchesToMeters(14.325)),
            new Rotation3d(0, Math.toRadians(23), Math.toRadians(180))
        );

    }

    public static enum FieldElement {
        SPEAKER(4, 7), AMP(5, 6), 
        STAGE_LEFT(11, 15), STAGE_RIGHT(12, 16), CENTER_STAGE(13, 14), 
        LOB_TARGET(new Pose3d(AMP.redPose.interpolate(SPEAKER.redPose, 0.2).toPose2d()),
                   new Pose3d(AMP.bluePose.interpolate(SPEAKER.bluePose, 0.2).toPose2d())
        ),
        POLE(), //for activity fair
        CARPET(), 
        NOTE_3(new Translation3d(FieldConstants.maxX - FieldConstants.metersFromAllianceWallToFrontlineNotes, FieldConstants.maxY / 2.0, 0),
               new Translation3d(FieldConstants.metersFromAllianceWallToFrontlineNotes, FieldConstants.maxY / 2.0, 0)
        ),
        NOTE_2(NOTE_3.redPose.getTranslation().plus(new Translation3d(0, FieldConstants.metersBetweenFrontlineNotes, 0)),
               NOTE_3.bluePose.getTranslation().plus(new Translation3d(0, FieldConstants.metersBetweenFrontlineNotes, 0))
        ),
        NOTE_1(NOTE_2.redPose.getTranslation().plus(new Translation3d(0, FieldConstants.metersBetweenFrontlineNotes, 0)),
               NOTE_2.bluePose.getTranslation().plus(new Translation3d(0, FieldConstants.metersBetweenFrontlineNotes, 0))
        ),
        NOTE_6(new Translation3d(FieldConstants.maxX / 2.0, FieldConstants.maxY / 2.0, 0)),
        NOTE_5(NOTE_6.redPose.getTranslation().plus(new Translation3d(0, FieldConstants.metersBetweenMidlineNotes, 0))),
        NOTE_4(NOTE_5.redPose.getTranslation().plus(new Translation3d(0, FieldConstants.metersBetweenMidlineNotes, 0))),
        NOTE_7(NOTE_6.redPose.getTranslation().plus(new Translation3d(0, -FieldConstants.metersBetweenMidlineNotes, 0))),
        NOTE_8(NOTE_7.redPose.getTranslation().plus(new Translation3d(0, -FieldConstants.metersBetweenMidlineNotes, 0))),
        MID_FIELD(new Translation3d(FieldConstants.maxX / 2.0, FieldConstants.maxY / 2.0, 0)),
        WING(new Translation3d(FieldConstants.maxX - FieldConstants.metersFromAllianceWallToWing, FieldConstants.maxY / 2.0, 0),
             new Translation3d(FieldConstants.metersFromAllianceWallToWing, FieldConstants.maxY / 2.0, 0)),
        SOURCE(9, 2);

        private static class FieldConstants {
            public static final double maxX = VisionConstants.aprilTagFieldLayout.getFieldLength();
            public static final double maxY = VisionConstants.aprilTagFieldLayout.getFieldWidth();
            public static final double metersBetweenFrontlineNotes = Units.inchesToMeters(57);
            public static final double metersBetweenMidlineNotes = Units.inchesToMeters(66);
            public static final double metersFromAllianceWallToFrontlineNotes = Units.inchesToMeters(114);
            public static final double metersFromAllianceWallToWing = Units.inchesToMeters(231.20);
        }

        /* End of Enum Instances */

        private Pose3d redPose;
        private Pose3d bluePose;
        public static Translation3d demoTargetLocation;

        private FieldElement(int redTagID, int blueTagID) {
            this.redPose = VisionConstants.aprilTagFieldLayout.getTagPose(redTagID).get();
            this.bluePose = VisionConstants.aprilTagFieldLayout.getTagPose(blueTagID).get();

            /* Target the opening of the speaker, rather than the speaker tag */
            if (redTagID == 4 || blueTagID == 7) {
                double speakerUpperLipHeightMeters = Units.inchesToMeters(82.90);
                double speakerLowerLipHeightMeters = Units.inchesToMeters(78.13);
                double speakerDepthIntoFieldMeters = Units.inchesToMeters(18.11);
                double speakerHeightMeters = (speakerUpperLipHeightMeters + speakerLowerLipHeightMeters) / 2.;

                double speakerY = Units.inchesToMeters(218.42);
                double redSpeakerX = Units.inchesToMeters(652.73-1.5) - (speakerDepthIntoFieldMeters / 2.);
                double blueSpeakerX = 0 + (speakerDepthIntoFieldMeters / 2.);

                Translation3d redLocation = new Translation3d(redSpeakerX, speakerY, speakerHeightMeters);
                Rotation3d redOrientation = new Rotation3d(0 , 0, Math.toRadians(180));
                Translation3d blueLocation = new Translation3d(blueSpeakerX, speakerY, speakerHeightMeters);
                Rotation3d blueOrientation = new Rotation3d(0, 0, 0);

                redPose = new Pose3d(redLocation, redOrientation);
                bluePose = new Pose3d(blueLocation, blueOrientation);
            }

            // use the middle of the source as the pose
            if (redTagID == 9 || redTagID == 10 || blueTagID == 1 || blueTagID == 2) {
                Pose3d firstRedPose = VisionConstants.aprilTagFieldLayout.getTagPose(9).get();
                Pose3d secondRedPose = VisionConstants.aprilTagFieldLayout.getTagPose(10).get();

                Pose3d firstBluePose = VisionConstants.aprilTagFieldLayout.getTagPose(1).get();
                Pose3d secondBluePose = VisionConstants.aprilTagFieldLayout.getTagPose(2).get();

                this.redPose = firstRedPose.interpolate(secondRedPose, 0.5);
                this.bluePose = firstBluePose.interpolate(secondBluePose, 0.5);
            }
        }

        private FieldElement(Translation3d location) {
            this.redPose = new Pose3d(location, new Rotation3d());
            this.bluePose = new Pose3d(location, new Rotation3d());
        }

        private FieldElement(Translation3d redLocation, Translation3d blueLocation) {
            this.redPose = new Pose3d(redLocation, new Rotation3d());
            this.bluePose = new Pose3d(blueLocation, new Rotation3d());
        }

        private FieldElement(Pose3d redPose, Pose3d bluePose) {
            this.redPose = redPose;
            this.bluePose = bluePose;
        }

        private FieldElement() {
            // ONLY TO BE USED FOR FieldElement.CARPET,
            // because the location of the shart depends on the location of the robot.
            this.redPose = null;
            this.bluePose = null;
        }

        public Pose3d getPose() {
            Optional<Alliance> alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
                if (alliance.get() == Alliance.Red) {
                    return redPose;
                }
                if (alliance.get() == Alliance.Blue) {
                    return bluePose;
                }
            }
            // Should never get to this point as long as we're connected to the driver station.
            return new Pose3d();
        }

        public Translation3d getLocation() {
            return getPose().getTranslation();
        }

        public Rotation3d getOrientation() {
            return getPose().getRotation();
        }

        public double getX() {
            return getLocation().getX();
        }

        public double getY() {
            return getLocation().getY();
        }

        public double getZ() {
            return getLocation().getZ();
        }

        public static Pose2d getClosestTrap(Pose2d yourPoseOnTheField) {
            Pose2d[] trapLocations = {STAGE_LEFT.getPose().toPose2d(),
                                      STAGE_RIGHT.getPose().toPose2d(),
                                      CENTER_STAGE.getPose().toPose2d()};

            return yourPoseOnTheField.nearest(Arrays.asList(trapLocations));
        }

        public static int getSpeakerTagID() {
            if (Constants.isDemoMode) {
                return 4;
            }

            Optional<Alliance> alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
                if (alliance.get() == Alliance.Red) {
                    return 4;
                }
                if (alliance.get() == Alliance.Blue) {
                    return 7;
                }
            }
            // Should never get to this point as long as we're connected to the driver station.
            return -1;
        }
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
