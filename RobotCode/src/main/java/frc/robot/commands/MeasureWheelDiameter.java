package frc.robot.commands;

import java.util.Arrays;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.SwerveModuleConstants;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class MeasureWheelDiameter extends Command {

    private Drivetrain drivetrain;
    private double[] startWheelPositionsMeters = {0, 0, 0, 0};
    private Rotation2d startRotation;

    public MeasureWheelDiameter(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        super.addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        drivetrain.setPoseMeters(new Pose2d());

        startRotation = drivetrain.getGyroRotation2d();

        for (int i = 0; i < drivetrain.getModulePositions().length; i += 1) {
            startWheelPositionsMeters[i] = drivetrain.getModulePositions()[i].distanceMeters;
        }
    }

    @Override
    public void execute() {
        drivetrain.robotOrientedDrive(
            new ChassisSpeeds(
                0,
                0,
                1
            ), true);

    }

    @Override
    public void end(boolean isInterrupted) {
        drivetrain.fieldOrientedDrive(new ChassisSpeeds(), true);

        //actual distance traveled by each wheel in meters
        double[] actualTravelMeters = {0, 0, 0, 0};
        //average of the actual distances
        double averageTravelMeters = 0.;

        //rotation the robot did according to the gryo
        double rotationAngleRadians = drivetrain.getGyroRotation2d().getRadians() - startRotation.getRadians();
        //theoretical distance calculated based off of rotations of the robot
        double theoreticalDistanceMeters = DrivetrainConstants.drivetrainRadiusMeters*rotationAngleRadians;
        
        for (int i = 0; i < drivetrain.getModulePositions().length; i += 1) {
            actualTravelMeters[i] = Math.abs(drivetrain.getModulePositions()[i].distanceMeters - startWheelPositionsMeters[i]);

            averageTravelMeters += 0.25*actualTravelMeters[i];
        }

        //multiply our current measurement of wheel diameter by (theoretical value)/(actual value)
        double recalculatedRadiusMeters = SwerveModuleConstants.wheelRadiusMeters*(theoreticalDistanceMeters/averageTravelMeters);

        System.out.println("Drivetrain Calibration Complete!");
        System.out.println("Total Rotation (rad): " + Double.toString(rotationAngleRadians));
        System.out.println("Wheel travel distances using CURRENT radius (m)" + Arrays.toString(actualTravelMeters));
        System.out.println("***Recalculated Wheel Radius*** (m): " + Double.toString(recalculatedRadiusMeters));
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
