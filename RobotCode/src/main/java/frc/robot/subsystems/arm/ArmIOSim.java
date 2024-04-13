// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.lib.MotorTempObject;
import frc.robot.Constants.ArmConstants;

/** Add your docs here. */
public class ArmIOSim implements ArmIO {

    private SingleJointedArmSim armSim;

    public ArmIOSim() {
        armSim = new SingleJointedArmSim(
            DCMotor.getNEO(2), 
            ArmConstants.armGearReduction,
            0.658,
            0.483, 
            Math.toRadians(ArmConstants.armMinAngleDegrees), 
            Math.toRadians(ArmConstants.armMaxAngleDegrees), 
            true, 
            Math.toRadians(0));
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        armSim.update(0.02);

        inputs.armAngleDegrees = Math.toDegrees(armSim.getAngleRads());
        inputs.leftEncoderReadingDegrees = Math.toDegrees(armSim.getAngleRads());
        inputs.rightEncoderReadingDegrees = Math.toDegrees(armSim.getAngleRads());
        inputs.armVelocityDegreesPerSecond = Math.toDegrees(armSim.getVelocityRadPerSec());

        inputs.atLowerLimit = armSim.hasHitLowerLimit();
        inputs.atUpperLimit = armSim.hasHitUpperLimit();
    }

    @Override
    public List<MotorTempObject> getMotorTemps() {
        ArrayList<MotorTempObject> temps = new ArrayList<MotorTempObject>();
        temps.add(new MotorTempObject("leftPivot", 25));
        temps.add(new MotorTempObject("rightPivot", 25));
        return temps;
    }

    @Override
    public void setArmMotorVolts(double volts) {
        armSim.setInputVoltage(volts);
    }
}
