// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Add your docs here. */
public class Intake extends SubsystemBase {

    DigitalInput intakeProximitySwitch;

    public Intake() {
        intakeProximitySwitch = new DigitalInput(0);
    }

    public boolean isRingInIntakeBottom() {
        return !intakeProximitySwitch.get();
    }



    @Override
    public void periodic() {
        Logger.recordOutput("intake/intakeProximitySwitch.get()", intakeProximitySwitch.get());
    }

}
