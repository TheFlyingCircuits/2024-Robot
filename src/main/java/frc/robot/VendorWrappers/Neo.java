package frc.robot.VendorWrappers;

import java.util.function.Supplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkRelativeEncoder.Type;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;

public class Neo extends CANSparkMax {

    private String name;

    private RelativeEncoder encoder;
    private double mostRecentGoodPosition = 0;
    private double mostRecentGoodVelocity = 0;
    private double mostRecentGoodAppliedOutput = 0;
    private double mostRecentGoodBusVoltage = 0;
    private double mostRecentGoodOutputCurrent = 0;


    // physics constants
    /** How many volts were applied to the motor by the manufacturer when determining its electrical characteristics. */
    public static final double nominalVoltage = 12;

    /** How quickly the motor turns (radians per second) when 12 volts are applied and there is no load on the motor. */
    public static final double freeSpeed = Units.rotationsPerMinuteToRadiansPerSecond(5676);

    /** How much current (amps) flowed through the motor while spinning at the free speed */
    public static final double freeCurrent = 1.8;

    /** How much torque (newton-meters) the stator exerts on the rotor when the rotor is held still,
     *  and then 12 volts are applied to the motor. */
    public static final double stallTorque = 2.6;

    /** How much current (amps) flows through the motor coils when the rotor is held still, and then 12 volts are applied to the motor. */
    public static final double stallCurrent = 105;

    /** How much torque (newton-meters) the stator will exert on the rotor per amp of current that flows through the motor coils.  */
    public static final double torquePerAmp = stallTorque / stallCurrent;

    /** How much resistance (ohms) the motor windings have. */
    public static final double windingResistance = nominalVoltage / stallCurrent;

    /** How many volts are induced in the motor windings by the permanent magnets on the rotor
     *  for each radian-per-second that the rotor is spinning at. */
    public static final double kEMF = (nominalVoltage - (freeCurrent * windingResistance)) / freeSpeed;

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

    public void exertTorque(double newtonMeters) {
        double rpm = this.getVelocity();
        double radiansPerSecond = Units.rotationsPerMinuteToRadiansPerSecond(rpm);
        double inducedVolts = -radiansPerSecond * kEMF;
        // T = kT * I = kT * (V/R) = kT * (Vapp + Vinduced) / R
        // vApp = T/kT * R - vInduced
        double volts = ((newtonMeters / torquePerAmp) * windingResistance) - inducedVolts;
        // double amps = newtonMeters / torquePerAmp;
        // super.getPIDController();
        // super.getOutputCurrent();
        // super.enableVoltageCompensation(nominalVoltage);
        // super.getPIDController().setReference(amps, ControlType.kCurrent);
        super.setVoltage(volts);
    }


    
}
