package frc.robot.commands;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldElement;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class AimEverythingAtTrap extends Command {

    Drivetrain drivetrain;

    public AimEverythingAtTrap(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        
        super.addRequirements(drivetrain);
    }

    public Command getPathFollowingCommand() {
        return AutoBuilder.followPath(getPathToClosestTrap());
    }

    public PathPlannerPath getPathToClosestTrap() {
        Translation2d startLocation = drivetrain.getPoseMeters().getTranslation();
        Translation2d endLocation = this.getClosestTrap().getTranslation();
        Rotation2d directionOfTravel = endLocation.minus(startLocation).getAngle();
        List<Translation2d> waypoints = PathPlannerPath.bezierFromPoses(
            new Pose2d(startLocation, directionOfTravel), new Pose2d(endLocation, directionOfTravel)
        );

        // apriltag angle points in the direction the tag itself is facing.
        // the tag faces away from the trap, and we want the robot to face
        // away from the trap as well because we approach using the back of the robot.
        Rotation2d desiredRotation = this.getClosestTrap().getRotation();

        double safetyFactor = 1./1.;
        double maxSpeed = 4.0 * safetyFactor;
        double maxAccel = 4.0 * safetyFactor;
        double maxAngularSpeed = 2 * Math.PI * safetyFactor;
        double maxAngularAccel = 2 * Math.PI * safetyFactor;

        PathPlannerPath pathToTrap = new PathPlannerPath(waypoints,
        new PathConstraints(maxSpeed, maxAccel, maxAngularSpeed, maxAngularAccel),
        new GoalEndState(0, desiredRotation, true));

        pathToTrap.preventFlipping = true;
        return pathToTrap;
    }

    public Pose2d getClosestTrap() {
        Translation2d robotPosition = drivetrain.getPoseMeters().getTranslation();

        Pose2d[] trapLocaitons = {drivetrain.getLocationOfFieldElement(FieldElement.STAGE_LEFT),
                                  drivetrain.getLocationOfFieldElement(FieldElement.STAGE_RIGHT),
                                  drivetrain.getLocationOfFieldElement(FieldElement.CENTER_STAGE)};

        Pose2d closestTrap = trapLocaitons[0];
        for (int i = 1; i < trapLocaitons.length; i += 1) {
            Pose2d currentTrap = trapLocaitons[i];

            double distanceToCurrent = robotPosition.getDistance(currentTrap.getTranslation());
            double distanceToClosest = robotPosition.getDistance(closestTrap.getTranslation());

            if (distanceToCurrent < distanceToClosest) {
                closestTrap = currentTrap;
            }
        }

        return closestTrap;
    }
}
