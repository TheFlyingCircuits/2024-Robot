package frc.robot.subsystems;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants.*;


public class ShooterIOSim implements ShooterIO {
    private FlywheelSim leftSim = new FlywheelSim(
        DCMotor.getKrakenX60(1),
        ShooterConstants.kFlywheelGearReduction,
        0.03
    );
    private FlywheelSim rightSim = new FlywheelSim(
        DCMotor.getKrakenX60(1),
        ShooterConstants.kFlywheelGearReduction,
        0.03
    );

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        leftSim.update(0.02);
        rightSim.update(0.02);

        inputs.leftFlywheelsRPM = leftSim.getAngularVelocityRPM();
        inputs.rightFlywheelsRPM = rightSim.getAngularVelocityRPM();
    }

    @Override
    public void setLeftMotorVolts(double volts) {
        leftSim.setInputVoltage(volts);
    }

    @Override
    public void setRightMotorVolts(double volts) {
        rightSim.setInputVoltage(volts);
    }
}
