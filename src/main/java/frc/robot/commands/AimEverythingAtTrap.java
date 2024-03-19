package frc.robot.commands;

import java.util.Arrays;
import java.util.List;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FlyingCircuitUtils;
import frc.robot.Constants.FieldElement;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class AimEverythingAtTrap extends Command {

    private Drivetrain drivetrain;
    private Supplier<ChassisSpeeds> translationController;

    public AimEverythingAtTrap(Drivetrain drivetrain, Supplier<ChassisSpeeds> translationController) {
        this.drivetrain = drivetrain;
        this.translationController = translationController;
        
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

        double safetyFactor = 1./2.;
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

    public void execute() {
        drivetrain.fieldOrientedDriveOnALine(translationController.get(), getClosestTrap());
    }

    public Pose2d getClosestTrap() {
        Pose2d[] trapLocations = {FlyingCircuitUtils.getLocationOfFieldElement(FieldElement.STAGE_LEFT),
                                  FlyingCircuitUtils.getLocationOfFieldElement(FieldElement.STAGE_RIGHT),
                                  FlyingCircuitUtils.getLocationOfFieldElement(FieldElement.CENTER_STAGE)};

        return drivetrain.getPoseMeters().nearest(Arrays.asList(trapLocations));
    }
}
