package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.DrivetrainConstants;

/**
 * A wrapper class for driver controls
 */
public class HumanDriver {

    private CommandXboxController controller;

    public HumanDriver(int xboxControllerPort) {
        controller = new CommandXboxController(xboxControllerPort);
    }

    public CommandXboxController getXboxController() {
        return controller;
    }

    public ChassisSpeeds getRequestedFieldOrientedVelocity() {
        return this.getRequestedRobotVelocity(true);
    }

    public ChassisSpeeds getRequestedRobotOrientedVelocity() {
        return this.getRequestedRobotVelocity(false);
    }

    /** Generates a command to rumble the controller for a given duration and strength.
     * @param seconds - Time to rumble the controller for, in seconds.
     * @param strength - Strength to rumble the controller at, from 0 to 1.
     */
    public Command rumbleController(double seconds, double strength) {
        return new InstantCommand(() -> controller.getHID().setRumble(RumbleType.kBothRumble, strength))
            .andThen(new WaitCommand(seconds))
            .andThen(new InstantCommand(() -> controller.getHID().setRumble(RumbleType.kBothRumble, 0)));
    }

    private ChassisSpeeds getRequestedRobotVelocity(boolean fieldRelative) {
        double rawThrottleY = -controller.getLeftX(); // Positive robot y is to the left, which is negative controller x
        double rawThrottleX = -controller.getLeftY(); // positive robot x is forward, which is negative controller y (for aircraft reasons)
        double angle = Math.atan2(rawThrottleY, rawThrottleX);

        // Apply deadzone and enforce max throttle
        double totalThrottle = Math.sqrt(rawThrottleX*rawThrottleX + rawThrottleY*rawThrottleY);
        if (totalThrottle < ControllerConstants.controllerDeadzone) {
            totalThrottle = 0;
        }
        if (totalThrottle > 1) {
            totalThrottle = 1; // stickX and stickY both range from [-1, 1], and can
                               // sometimes have a net magnitude that is greater than 1
        }

        // Increase sensitivity at slower speeds
        // adjust this power value for diffferences in how the robot handles (recommended between 1.5 and 3)
        // ^ old comment from original JoystickDrive command
        totalThrottle = Math.pow(totalThrottle, 2.3);
        double desiredSpeed = totalThrottle * DrivetrainConstants.maxDesiredTeleopVelocityMetersPerSecond;

        // Get desired rotation speed from right stick
        double angularThrottle = -controller.getRightX();
        if (Math.abs(angularThrottle) < ControllerConstants.controllerDeadzone) {
            angularThrottle = 0;
        }
        if (Math.abs(angularThrottle) > 1.0) {
            angularThrottle = Math.signum(angularThrottle) * 1.0;
        }
        angularThrottle = Math.signum(angularThrottle) * Math.pow(Math.abs(angularThrottle), 2.3);
        double desiredAngularSpeed = angularThrottle * DrivetrainConstants.maxDesiredTeleopAngularVelocityRadiansPerSecond;

        ChassisSpeeds requestedVelocity = new ChassisSpeeds();
        requestedVelocity.vxMetersPerSecond = desiredSpeed * Math.cos(angle);
        requestedVelocity.vyMetersPerSecond = desiredSpeed * Math.sin(angle);
        requestedVelocity.omegaRadiansPerSecond = desiredAngularSpeed;

        if (fieldRelative) {
            // Field relative velocities are specified from the blue alliance perspective
            // so they need to be flipped if on red alliance
            Optional<Alliance> alliance = DriverStation.getAlliance();
            if (alliance.isPresent() && alliance.get() == Alliance.Red) {
                requestedVelocity.vxMetersPerSecond *= -1; // forward on blue is +blueX, forward on red is -blueX
                requestedVelocity.vyMetersPerSecond *= -1; // left on blue is +blueY, left on red is -blueY
            }
        }

        return requestedVelocity;
    }
    
}
