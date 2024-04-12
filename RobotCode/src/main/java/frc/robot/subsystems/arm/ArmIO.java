// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.AutoLog;

import frc.lib.MotorTempObject;
public interface ArmIO {

    @AutoLog
    public class ArmIOInputs {
        /**
         * Angle of the arm in degrees. An angle of 0 represents a perfectly horizontal, forward facing arm.
         * Raising the arm from this position will make the angle positive, while lowering it will be negative.
         */
        public double armAngleDegrees = 0.0;

        /**
         * Angle that the left encoder is reading, in degrees.
         * If nothing is broken, this should be very similar to armAngleDegrees.
         */
        public double leftEncoderReadingDegrees = 0.0;

        /**
         * Angle that the right encoder is reading, in degrees.
         * If nothing is broken, this should be very similar to armAngleDegrees.
         */
        public double rightEncoderReadingDegrees = 0.0;

        /**
         * Velocity of the arm in degrees per second.
         * Follows the same direction as armAngleDegrees.
         */
        public double armVelocityDegreesPerSecond = 0.0;

        public double leftMotorAppliedVoltage = 0.0;
        public double rightMotorAppliedVoltage = 0.0;

        public boolean atUpperLimit = false;
        public boolean atLowerLimit = false;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(ArmIOInputs inputs) {};

    /** 
     * Run the two arm motors at the specified voltage.
     * A positive value will raise the arm, while a negative value will lower it.
    */
    public default void setArmMotorVolts(double volts) {};

    public default void setArmEncoderPosition(double degrees) {};

    public default void setCoast(boolean makeCoast) {};

    public default boolean isCoast() {return false;};

    public default List<MotorTempObject> getMotorTemps() {return new ArrayList<MotorTempObject>();};

}
