package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    public class ShooterIOInputs {
        public double leftFlywheelsRotationsPerSecond = 0.0;
        public double rightFlywheelsRotationsPerSecond = 0.0;
    }

    public default void updateInputs(ShooterIOInputs inputs) {};

    /** Run the shooter's left motor at the specified volts. */
    public default void setLeftMotorVolts(double volts) {};

    /** Run the shooter's right motor at the specified volts. */
    public default void setRightMotorVolts(double volts) {};

}
