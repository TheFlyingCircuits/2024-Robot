package frc.robot.VendorWrappers;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkRelativeEncoder.Type;

import edu.wpi.first.wpilibj.Timer;
import frc.lib.subsystem.Fault;

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

    public REVLibError waitForConfig(Supplier<REVLibError> configFunction, String errorMessage) {
        double waitTimeSeconds = 1.0/8.0;
        
        while (true) {
            REVLibError errorCode = configFunction.get();

            if (errorCode == REVLibError.kOk) {
                return errorCode;
            }

            String fullErrorMessage = errorMessage;
            fullErrorMessage += "\nNeo Error: " + errorCode;
            fullErrorMessage += "\nRetrying...";
            System.out.println(fullErrorMessage);
            Timer.delay(waitTimeSeconds);
        }
    }

    public REVLibError restoreFactoryDefaults() {
        String errorMessage = "Failed to restore "+name+" to factory defaults!";
        return this.waitForConfig(super::restoreFactoryDefaults, errorMessage);
    }

    public REVLibError setIdleMode(IdleMode idleMode) {
        String errorMessage = "Failed to set "+name+"'s idle mode to "+idleMode+"!";
        return this.waitForConfig(() -> {return super.setIdleMode(idleMode);}, errorMessage);
    }

    public void setInverted(boolean isInverted) {
        String errorMessage = "Failed to set "+name+" to be ";
        if (isInverted) {
            errorMessage += "Clockwise Positive";
        } else {
            errorMessage += "Counter Clockwise Positive";
        }

        this.waitForConfig(() -> {super.setInverted(isInverted); return super.getLastError(); /* idk if this works */}, errorMessage);
    }

    public REVLibError setSmartCurrentLimit(int limitAmps) {
        String errorMessage = "Failed to set a current limit of "+limitAmps+" amps for "+name+"!";
        return this.waitForConfig(() -> {return super.setSmartCurrentLimit(limitAmps);}, errorMessage);
    }

    public REVLibError burnFlash() {
        String errorMessage = "Failed to burn settings to flash for "+name+"!";
        return this.waitForConfig(super::burnFlash, errorMessage);
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
        String errorMessage = "Failed to set "+name+"'s positionConversionFactor to "+factor+"!";
        return this.waitForConfig(() -> {return encoder.setPositionConversionFactor(factor);}, errorMessage);
    }

    public REVLibError setVelocityConversionFactor(double factor) {
        String errorMessage = "Failed to set "+name+"'s velocityConversionFactor to "+factor+"!";
        return this.waitForConfig(() -> {return encoder.setVelocityConversionFactor(factor);}, errorMessage);
    }

    public REVLibError setPosition(double position) {
        String errorMessage = "Failed to set the position of "+name+" to "+position+"!";
        return this.waitForConfig(() -> {return encoder.setPosition(position);}, errorMessage);
    }


        public List<Fault> autoDiagnoseIsAtTargetRPS(double expectedRPS, double tolerance, boolean isForward) {
        ArrayList<Fault> faults = new ArrayList<Fault>();

        String direction = isForward ? "forward" : "backward";

        if (Math.abs(this.getVelocity()) < .1) {
            new Fault("[Auto Diagnose] "+name+" not moving " +direction, false);
        } else if (Math.abs(this.getVelocity()) > expectedRPS+tolerance) {
            new Fault("[Auto Diagnose] "+name+" moving "+direction+" too fast", false);
        } else if (Math.abs(this.getVelocity()) < expectedRPS-tolerance) {
            new Fault("[Auto Diagnose] "+name+" moving "+direction+" too slow", false);
        }
        return faults;
    }
    


    
}
