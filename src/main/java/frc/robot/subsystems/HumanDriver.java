package frc.robot.subsystems;

import java.util.Optional;

import javax.swing.GroupLayout.Alignment;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.DrivetrainConstants;

/**
 * A wrapper class for driver controls
 */
public class HumanDriver {

    private CommandXboxController controller;
    private static HumanDriver charlie = new HumanDriver(0);

    private HumanDriver(int xboxControllerPort) {
        controller = new CommandXboxController(xboxControllerPort);
    }

    public CommandXboxController getXboxController() {
        return controller;
    }

    public static HumanDriver getCharlie() {
        return charlie;
    }

    public ChassisSpeeds getRequestedFieldOrientedVelocity() {
        return this.getRequestedRobotVelocity(true);
    }

    public ChassisSpeeds getRequestedRobotVelocity(boolean fieldRelative) {
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
