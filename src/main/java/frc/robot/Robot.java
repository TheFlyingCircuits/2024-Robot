// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.reflect.Field;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.FieldElement;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
    private Command m_autonomousCommand;

    private RobotContainer m_robotContainer;

    private SendableChooser<FieldElement> firstPriorityChooser = initNoteChooser("First Priority: ");
    private SendableChooser<FieldElement> secondPriorityChooser = initNoteChooser("Second Priority: ");
    private SendableChooser<FieldElement> thirdPriorityChooser = initNoteChooser("Third Priority: ");
    private SendableChooser<FieldElement> startingLocationChooser;

    private FieldElement chosenStartingLocation;
    private FieldElement[] notesToGoFor = {FieldElement.NOTE_8, FieldElement.NOTE_7, FieldElement.NOTE_6};

    private SendableChooser<FieldElement> initNoteChooser(String name) {
        SendableChooser<FieldElement> output = new SendableChooser<FieldElement>();

        output.addOption(name+FieldElement.NOTE_4.name(), FieldElement.NOTE_4);
        output.addOption(name+FieldElement.NOTE_5.name(), FieldElement.NOTE_5);
        output.addOption(name+FieldElement.NOTE_6.name(), FieldElement.NOTE_6);
        output.addOption(name+FieldElement.NOTE_7.name(), FieldElement.NOTE_7);
        output.addOption(name+FieldElement.NOTE_8.name(), FieldElement.NOTE_8);
        output.setDefaultOption(name+FieldElement.NOTE_6.name(), FieldElement.NOTE_6);

        return output;
    }


    private void initAdvantageKit() {
        Logger.recordMetadata("projectName", "2024Robot");
        Logger.addDataReceiver(new NT4Publisher());
        if (Constants.atCompetition) {
            Logger.addDataReceiver(new WPILOGWriter()); // <- log to USB stick
        }
        new PowerDistribution(); // Apparently just constructing a PDH
                                 // will allow it's values to be logged? 
                                 // This is what the advantage kit docs imply at least.
        Logger.start();
    }

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {

        initAdvantageKit();

        // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        // autonomous chooser on the dashboard.
        m_robotContainer = new RobotContainer();

        // Must buildAutoChooser() only after NamedCommmands have been registered in the RobotContainer constructor!
        startingLocationChooser = new SendableChooser<FieldElement>(); //AutoBuilder.buildAutoChooser();
        startingLocationChooser.addOption("Source Side", FieldElement.SOURCE);
        startingLocationChooser.addOption("Center Side", FieldElement.SPEAKER);
        startingLocationChooser.addOption("Amp Side", FieldElement.AMP);
        startingLocationChooser.setDefaultOption("Source Side", FieldElement.SOURCE);
        SmartDashboard.putData("Starting Location", startingLocationChooser);
        SmartDashboard.putData("First Priority", firstPriorityChooser);
        SmartDashboard.putData("Second Priority", secondPriorityChooser);
        SmartDashboard.putData("Third Priority", thirdPriorityChooser);

        DriverStation.silenceJoystickConnectionWarning(true);
        FollowPathCommand.warmupCommand().schedule();
        System.gc();
    }

    public void updateSelectedAuto() {
        FieldElement desiredStartingLocation = startingLocationChooser.getSelected();
        FieldElement[] desiredNotes = {firstPriorityChooser.getSelected(), secondPriorityChooser.getSelected(), thirdPriorityChooser.getSelected()};

        // Generate new auto if we need to
        boolean generateNewAuto = desiredStartingLocation != chosenStartingLocation;
        for (int i = 0; i < desiredNotes.length; i += 1) {
            generateNewAuto = generateNewAuto || (desiredNotes[i] != notesToGoFor[i]);

            // just don't generate a new auto if an impossible combo is requested!
            if (invalidAuto(desiredStartingLocation, desiredNotes[i])) {
                return;
            }
        }

        if (generateNewAuto) {

            chosenStartingLocation = desiredStartingLocation;
            for (int i = 0; i < desiredNotes.length; i += 1) {
                notesToGoFor[i] = desiredNotes[i];
            }

            if (desiredStartingLocation == FieldElement.SPEAKER) {
                m_autonomousCommand = m_robotContainer.centerSideAuto(notesToGoFor[0]);
                return;
            }
            else {
                m_autonomousCommand = m_robotContainer.sideAuto(notesToGoFor, chosenStartingLocation);
            }

            System.gc();
        }
    }

    private boolean invalidAuto(FieldElement startingLocation, FieldElement note) {
        return (startingLocation == FieldElement.SOURCE && !(note == FieldElement.NOTE_8 || note == FieldElement.NOTE_7 || note == FieldElement.NOTE_6)) ||
               (startingLocation == FieldElement.SPEAKER && !(note == FieldElement.NOTE_7 || note == FieldElement.NOTE_6 || note == FieldElement.NOTE_5)) ||
               (startingLocation == FieldElement.AMP && !(note == FieldElement.NOTE_6 || note == FieldElement.NOTE_5 || note == FieldElement.NOTE_4)) ||
               !(startingLocation == FieldElement.SOURCE || startingLocation == FieldElement.SPEAKER || startingLocation == FieldElement.AMP);
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

    /** This function is called once each time the robot enters Disabled mode. */
    @Override
    public void disabledInit() {
        //m_robotContainer.drivetrain.playOrchestra();
    }

    @Override
    public void disabledPeriodic() {
        m_robotContainer.drivetrain.setPoseToVisionMeasurement();
        if(!m_robotContainer.drivetrain.isSongPlaying()) {
            //m_robotContainer.drivetrain.playOrchestra();
        }

        updateSelectedAuto();
        if (m_autonomousCommand != null) {
            SmartDashboard.putString("Chosen Auto", m_autonomousCommand.getName());
        }
        else {
            SmartDashboard.putString("Chosen Auto", "Null");
        }
    }

    @Override
    public void disabledExit() {
        //m_robotContainer.drivetrain.stopOrchestra();
    }

    /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
    @Override
    public void autonomousInit() {
        // System.gc();
        m_robotContainer.drivetrain.setPoseToVisionMeasurement();

        //m_autonomousCommand = autoChooser.getSelected();

        // schedule the autonomous command (example)
        // if (m_autonomousCommand != null) {
        //     m_autonomousCommand.schedule();
        // }

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
