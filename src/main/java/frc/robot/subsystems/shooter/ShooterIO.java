package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    public class ShooterIOInputs {
        /**
         * Linear surface speed of the left set of flywheels.
         */
        public double leftFlywheelsMetersPerSecond = 0.0;
        /**
         * Linear surface speed of the right set of flywheels.
         */
        public double rightFlywheelsMetersPerSecond = 0.0;

        public double leftMotorAppliedVoltage = 0.0;
        public double rightMotorAppliedVoltage = 0.0;
        
        public double leftMotorOutputCurrent = 0.0;
        public double rightMotorOutputCurrent = 0.0;

        public double leftFlywheelRadians = 0.0;
        public double rightFlywheelRadians = 0.0;
    }

    public default void updateInputs(ShooterIOInputs inputs) {};

    /** Run the shooter's left motor at the specified volts. */
    public default void setLeftMotorVolts(double volts) {};

    /** Run the shooter's right motor at the specified volts. */
    public default void setRightMotorVolts(double volts) {};

    public default void setLeftMotorAmps(double amps) {};

    public default void setRightMotorAmps(double amps) {};
}
