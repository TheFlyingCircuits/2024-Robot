package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    public class ShooterIOInputs {
        public double leftFlywheelsRPM = 0.0;
        public double rightFlywheelsRPM = 0.0;
    }

    public default void updateInputs(ShooterIOInputs inputs) {};

    /** Run the shooter's left motor at the specified volts. */
    public default void setLeftMotorVolts(double volts) {};

    /** Run the shooter's right motor at the specified volts. */
    public default void setRightMotorVolts(double volts) {};

}
