package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Indexer;

/** Add your docs here. */
public class NoteTrackingIndexNote extends Command {


    private Drivetrain drivetrain;
    private Intake intake;
    private Indexer indexer;

    private Supplier<ChassisSpeeds> translationController;

    public NoteTrackingIndexNote(Intake intake, Indexer indexer, Drivetrain drivetrain, Supplier<ChassisSpeeds> translationController) {
        this.intake = intake;
        this.indexer = indexer;
        this.drivetrain = drivetrain;
        this.translationController = translationController;
        
        addRequirements(intake, indexer, drivetrain);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        intake.setVolts(12);
        indexer.setIndexerRPM(1100);


        if (drivetrain.intakeSeesNote()) {
            drivetrain.fieldOrientedDriveWhileAiming(
                translationController.get(),
                drivetrain.getFieldRelativeRotationToNote().getDegrees());
        }
        else {
            drivetrain.fieldOrientedDrive(translationController.get(), true);
        }

    }

    @Override
    public void end(boolean interrupted) {
        intake.setVolts(0);
        indexer.setVolts(0);
    }

    @Override
    public boolean isFinished() {
        return indexer.isNoteIndexed();
    }
}
