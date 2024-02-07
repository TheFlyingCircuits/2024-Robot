// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants.ArmConstants;

/** Add your docs here. */
public class ArmIOSim implements ArmIO {

    private SingleJointedArmSim armSim;

    public ArmIOSim() {
        armSim = new SingleJointedArmSim(
            DCMotor.getNEO(2), 
            ArmConstants.kArmGearReduction,
            0.468,
            0.508, 
            Math.toRadians(ArmConstants.kArmMinAngleDegrees), 
            Math.toRadians(ArmConstants.kArmMaxAngleDegrees), 
            true, 
            Math.toRadians(0));
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        armSim.update(0.02);

        inputs.armAngleDegrees = Math.toDegrees(armSim.getAngleRads());
        inputs.armVelocityDegreesPerSecond = Math.toDegrees(armSim.getVelocityRadPerSec());

        inputs.atLowerLimit = armSim.hasHitLowerLimit();
        inputs.atUpperLimit = armSim.hasHitUpperLimit();
    }

    @Override
    public void setArmMotorVolts(double volts) {
        armSim.setInputVoltage(volts);
    }
}
