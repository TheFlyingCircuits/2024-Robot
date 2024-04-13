package frc.robot.subsystems.shooter;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.lib.MotorTempObject;
import frc.robot.Constants.*;


public class ShooterIOSim implements ShooterIO {
    private FlywheelSim leftSim;
    private FlywheelSim rightSim;

    public ShooterIOSim() {

        leftSim = new FlywheelSim(
            DCMotor.getKrakenX60(1),
            ShooterConstants.flywheelGearReduction,
            0.000374);

        rightSim = new FlywheelSim(
            DCMotor.getKrakenX60(1),
            ShooterConstants.flywheelGearReduction,
            0.000374);
            
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        leftSim.update(0.02);
        rightSim.update(0.02);

        inputs.leftFlywheelsMetersPerSecond = leftSim.getAngularVelocityRPM()/60.*ShooterConstants.flywheelCircumferenceMeters;
        inputs.rightFlywheelsMetersPerSecond = rightSim.getAngularVelocityRPM()/60.*ShooterConstants.flywheelCircumferenceMeters;
    }

    @Override
    public void setLeftMotorVolts(double volts) {
        leftSim.setInputVoltage(volts);
    }

    @Override
    public void setRightMotorVolts(double volts) {
        rightSim.setInputVoltage(volts);
    }

        @Override
    public List<MotorTempObject> getMotorTemps() {
        ArrayList<MotorTempObject> temps = new ArrayList<MotorTempObject>();
        temps.add(new MotorTempObject("leftShooter", 25));
        temps.add(new MotorTempObject("rightShooter", 25));
        return temps;
    }
}
