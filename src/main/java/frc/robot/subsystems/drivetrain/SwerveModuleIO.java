package frc.robot.subsystems.drivetrain;

import org.littletonrobotics.junction.AutoLog;

public interface SwerveModuleIO {

    @AutoLog
    public class SwerveModuleIOInputs {
        public double drivePositionMeters = 0.0;
        public double driveVelocityMetersPerSecond = 0.0;
        
        public double angleAbsolutePositionDegrees = 0.0;

        public double driveAppliedVoltage = 0.0;
        public double driveCurrent = 0.0;
    }
    
    /** Updates the set of loggable inputs. */
    public default void updateInputs(SwerveModuleIOInputs inputs) {};

    /** Run the drive motor at the specified voltage. */
    public default void setDriveVoltage(double volts) {};

    /** Run the angle motor at the specified voltage. */
    public default void setAngleVoltage(double volts) {};

}
