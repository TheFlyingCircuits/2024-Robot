package frc.robot.subsystems.shooter;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.AutoLog;

import frc.lib.subsystem.Fault;

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
    }

    public default void updateInputs(ShooterIOInputs inputs) {};

    /** Run the shooter's left motor at the specified volts. */
    public default void setLeftMotorVolts(double volts) {};

    /** Run the shooter's right motor at the specified volts. */
    public default void setRightMotorVolts(double volts) {};

    public default List<Fault> getLeftFaults(double expectedRPS, double tolerance, boolean isForward) { return new ArrayList<Fault>(); };

    public default List<Fault> getRightFaults(double expectedRPS, double tolerance, boolean isForward) { return new ArrayList<Fault>(); };

}
