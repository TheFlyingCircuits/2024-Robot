package frc.robot.VendorWrappers;

import java.util.function.Supplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkRelativeEncoder.Type;

import edu.wpi.first.wpilibj.Timer;

public class Neo extends CANSparkMax {

    private String name;

    private RelativeEncoder encoder;
    private double mostRecentGoodPosition = 0;
    private double mostRecentGoodVelocity = 0;
    private double mostRecentGoodAppliedOutput = 0;
    private double mostRecentGoodBusVoltage = 0;
    private double mostRecentGoodOutputCurrent = 0;

    public Neo(int canID) {
        this("Neo #"+canID, canID);
    }

    public Neo(String name, int canID) {
        super(canID, MotorType.kBrushless);
        this.name = name;
        int encoderCountsPerMotorRev = 42; // comes from Neo Docs
        encoder = super.getEncoder(Type.kHallSensor, encoderCountsPerMotorRev);
        this.restoreFactoryDefaults();
    }

    public REVLibError waitForConfig(Supplier<REVLibError> configFunction) {
        double waitTimeSeconds = 1.0/8.0;
        
        while (true) {
            REVLibError errorCode = configFunction.get();

            if (errorCode == REVLibError.kOk) {
                System.out.println("Success!");
                return errorCode;
            }

            System.out.println("Neo Error: " + errorCode);
            System.out.println("Retrying...");
            Timer.delay(waitTimeSeconds);
        }
    }

    public REVLibError restoreFactoryDefaults() {
        System.out.println("Restoring "+name+" to factory defaults");
        return this.waitForConfig(super::restoreFactoryDefaults);
    }

    public REVLibError setIdleMode(IdleMode idleMode) {
        System.out.println("Setting "+name+" idle mode to "+idleMode);
        return this.waitForConfig(() -> {return super.setIdleMode(idleMode);});
    }

    public void setInverted(boolean isInverted) {
        if (isInverted) {
            System.out.println("Setting "+name+" to be Clockwise Positive");
        } else {
            System.out.println("Setting "+name+" to be Counter Clockwise Positive");
        }

        this.waitForConfig(() -> {super.setInverted(isInverted); return super.getLastError(); /* idk if this works */});
    }

    public REVLibError setSmartCurrentLimit(int limitAmps) {
        System.out.println("Setting current limit of "+limitAmps+" amps for "+name);
        return this.waitForConfig(() -> {return super.setSmartCurrentLimit(limitAmps);});
    }

    public REVLibError burnFlash() {
        return this.waitForConfig(super::burnFlash);
    }

    public double getPosition() {
        // this is how advantage kit odometry example does it.
        double newPosition = encoder.getPosition();
        REVLibError errorCode = super.getLastError();
        if (errorCode == REVLibError.kOk) {
            mostRecentGoodPosition = newPosition;
            return newPosition;
        }
        else {
            System.out.println("Error getting the position of "+name+": "+errorCode);
            System.out.println("Returning most recent valid position");
            return mostRecentGoodPosition;
        }
    }

    public double getVelocity() {
        // this is how advantage kit odometry example does it
        double newVelocity = encoder.getVelocity();
        REVLibError errorCode = super.getLastError();
        if (errorCode == REVLibError.kOk) {
            mostRecentGoodVelocity = newVelocity;
            return newVelocity;
        }
        else {
            System.out.println("Error getting the velocity of "+name+": "+errorCode);
            System.out.println("Returning most recent valid velocity");
            return mostRecentGoodVelocity;
        }
    }

    public double getAppliedOutput() {
        double newAppliedOutput = super.getAppliedOutput();
        REVLibError errorCode = super.getLastError();
        if (errorCode == REVLibError.kOk) {
            mostRecentGoodAppliedOutput = newAppliedOutput;
            return newAppliedOutput;
        }
        else {
            System.out.println("Error getting the applied output of "+name+": "+errorCode);
            System.out.println("Returning most recent valid applied output");
            return mostRecentGoodAppliedOutput;
        }
    }

    public double getBusVoltage() {
        double newBusVoltage = super.getBusVoltage();
        REVLibError errorCode = super.getLastError();
        if (errorCode == REVLibError.kOk) {
            mostRecentGoodBusVoltage = newBusVoltage;
            return newBusVoltage;
        }
        else {
            System.out.println("Error getting the bus voltage for "+name+": "+errorCode);
            System.out.println("Returning most recent valid bus voltage");
            return mostRecentGoodBusVoltage;
        }
    }

    public double getOutputCurrent() {
        double newOutputCurrent = super.getOutputCurrent();
        REVLibError errorCode = super.getLastError();
        if (errorCode == REVLibError.kOk) {
            mostRecentGoodOutputCurrent = newOutputCurrent;
            return newOutputCurrent;
        }
        else {
            System.out.println("Error getting the output current of "+name+": "+errorCode);
            System.out.println("Returning most recent valid output current");
            return mostRecentGoodOutputCurrent;
        }
    }

    public REVLibError setPositionConversionFactor(double factor) {
        System.out.println("Setting position conversion factor for "+name);
        return this.waitForConfig(() -> {return encoder.setPositionConversionFactor(factor);});
    }

    public REVLibError setVelocityConversionFactor(double factor) {
        System.out.println("Setting velocity conversion factor for "+name);
        return this.waitForConfig(() -> {return encoder.setVelocityConversionFactor(factor);});
    }

    public REVLibError setPosition(double position) {
        System.out.println("Setting position of "+name+" to "+position);
        return this.waitForConfig(() -> {return encoder.setPosition(position);});
    }


    
}
