// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.constant.DirectMethodHandleDesc;
import java.sql.Driver;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.LEDConstants;
import frc.robot.commands.leds.RedBlueChasePattern;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.leds.LEDs;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
    private Command m_autonomousCommand;

    private RobotContainer m_robotContainer;
    private Drivetrain drivetrain;

    private SendableChooser<Command> autoChooser;

    private void configAutoBuilder() {
        AutoBuilder.configureHolonomic(
            drivetrain::getPoseMeters, // Robot pose supplier
            drivetrain::setPoseMeters, // Method to reset odometry (will be called if your auto has a starting pose)
            drivetrain::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (ChassisSpeeds speeds) -> drivetrain.drive(speeds, true), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(4.0, 0.0, 0.0), // Rotation PID constants
                    DrivetrainConstants.maxAchievableVelocityMetersPerSecond, // Max module speed, in m/s
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
            drivetrain // Reference to this subsystem to set requirements
        );

        //TODO: set override when prep shot is active
        PPHolonomicDriveController.setRotationTargetOverride(null);
    }

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        // autonomous chooser on the dashboard.
        m_robotContainer = new RobotContainer();
        drivetrain = m_robotContainer.drivetrain;
        
        configAutoBuilder();
        

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        DriverStation.silenceJoystickConnectionWarning(true);
    }

    /**
     * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
     * that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic functions, but before LiveWindow and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();
    }

    Timer flashTimer = new Timer();
    Timer timeBetweenFlashes = new Timer();
    Timer slingshotTimer = new Timer();

    /** This function is called once each time the robot enters Disabled mode. */
    @Override
    public void disabledInit() {
        flashTimer.restart();
        timeBetweenFlashes.restart();
        slingshotTimer.restart();
    }


    @Override
    public void disabledPeriodic() {

        // leds.setLEDColor(LEDColor.ORANGE);
        //leds.chasePattern();


        m_robotContainer.arm.setDesiredPositionToCurrent();

        LEDs leds = m_robotContainer.leds;

        if (false) {
            double slinshotTime = 0.25;
            if (slingshotTimer.get() > 5.0) {
                slingshotTimer.restart();
            }
            if (slingshotTimer.get() < 2.0) {
                leds.solidColorHSV(LEDConstants.Hues.orangeSignalLight, 255, 255);
            }
            else if (slingshotTimer.get() < 2.0 + slinshotTime) {
                leds.slingshot(((slingshotTimer.get()-2)/slinshotTime) % 1.0);
            }
            else if (slingshotTimer.get() < 5.0) {
                leds.solidColorHSV(leds.getAllianceHue(), 255, 255);
            }
            // leds.slingshot((Timer.getFPGATimestamp()/0.25) % 1.0);
            return;
        }


        if (false) {
            int hue = (int) SmartDashboard.getNumber("hue", 0);
            leds.solidColorHSV(hue, 255, 255);
            SmartDashboard.putNumber("hue", hue);
            return;
        }

        if (false) {
            leds.loadingPattern();
            return;
        }
        
        if (false) {
            if (flashTimer.get() < 3.0) {
                leds.loadingPattern();
            }
            else if (flashTimer.get() < 3.5) {
                if (timeBetweenFlashes.get() < (1.0/12.0)) {
                    leds.solidColorHSV(0, 0, 255);
                }
                else if (timeBetweenFlashes.get() < (2.0/12.0)) {
                    leds.solidColorHSV(0, 0, 0);
                }
                else {
                    timeBetweenFlashes.restart();
                }
            }
            else {
                flashTimer.restart();
            }
            return;
        }

        double frequency = 1.0;
        double period = 1/frequency;
        double flywheelProgress = Math.sin(2*Math.PI*(Timer.getFPGATimestamp()/period))/2.0 + 0.5;
        double armProgress = Math.sin(2*Math.PI*(Timer.getFPGATimestamp()/period) + (2*Math.PI/3.0))/2.0 + 0.5;
        double drivetrainProgress = Math.sin(2*Math.PI*(Timer.getFPGATimestamp()/period) + (4*Math.PI/3.0))/2.0 + 0.5;
        leds.showFlywheelProgress(flywheelProgress);
        leds.showArmProgress(flywheelProgress);
        leds.showDrivetrainProgress(flywheelProgress);
    }

    /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
    @Override
    public void autonomousInit() {
        m_robotContainer.drivetrain.setPoseToVisionMeasurement();

        m_autonomousCommand = autoChooser.getSelected();

        // schedule the autonomous command (example)
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }

    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {}

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        // TODO: should we put this in disabledInit() too?
        CommandScheduler.getInstance().cancelAll();
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {}

    /** This function is called once when the robot is first started up. */
    @Override
    public void simulationInit() {}

    /** This function is called periodically whilst in simulation. */
    @Override
    public void simulationPeriodic() {}
}
