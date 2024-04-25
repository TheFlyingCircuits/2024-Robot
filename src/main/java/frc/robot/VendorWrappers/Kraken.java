package frc.robot.VendorWrappers;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

/**
 * A wrapper class around TalonFX that provides some convience features,
 * like auto-retyring configs if they fail.
 */
public class Kraken extends TalonFX {

    // physics constants
    /** How many volts were applied to the motor by the manufacturer when determining its electrical characteristics. */
    public static final double nominalVoltage = 12;

    /** How quickly the motor turns (radians per second) when the nominal voltage is applied and there is no load on the motor. */
    public static final double freeSpeed = Units.rotationsPerMinuteToRadiansPerSecond(5800);

    /** How much current (amps) flowed through the motor while spinning at the free speed */
    public static final double freeCurrent = 2.0;

    /** How much torque (newton-meters) the stator exerts on the rotor when the rotor is held still,
     *  and then the nominal voltage is applied to the motor. */
    public static final double stallTorque = 9.37;

    /** How much current (amps) flowed through the motor coils when the rotor was held still, and then the nominal voltage was applied to the motor. */
    public static final double stallCurrent = 483;

    /** How much torque (newton-meters) the stator will exert on the rotor per amp of current that flows through the motor coils.  */
    public static final double torquePerAmp = stallTorque / stallCurrent;

    /** How much resistance (ohms) the motor windings have. */
    public static final double windingResistance = nominalVoltage / stallCurrent;

    /** How many volts are induced in the motor windings by the permanent magnets on the rotor
     *  for each radian-per-second that the rotor is spinning at. */
    public static final double kEMF = (nominalVoltage - (freeCurrent * windingResistance)) / freeSpeed;

    /** How we should refer to this motor in error messages. */
    private String name;

    public Kraken(int canID, String canBusName) {
        this("Kraken #"+canID, canID, canBusName);
    }

    public Kraken(String motorName, int canID, String canBusName) {
        super(canID, canBusName);
        name = motorName;
    }
    
    
    public void applyConfig(TalonFXConfiguration config) {
        /** Check that we've set some critical safety configs */
        double statorCurrentLimit = config.CurrentLimits.StatorCurrentLimit;
        boolean currentLimitEnabled = config.CurrentLimits.StatorCurrentLimitEnable;
        while (statorCurrentLimit == 0) {
            System.out.println("bruh, u forgot to set a current limit for "+name);
            System.out.println("u gotta set something for TalonFXConfiguration.CurrentLimits.StatorCurrentLimit");
            Timer.delay(5);
        }
        while (!currentLimitEnabled) {
            System.out.println("bruh, u forgot to enable the current limit for "+name);
            System.out.println("u gotta explicitly set TalonFXConfiguration.CurrentLimits.StatorCurrentLimitEnable to true");
            Timer.delay(5);
        }

        /** Print a warning if you're running a test and don't want a current limit to interfere. */
        if (statorCurrentLimit >= 100) { 
            System.out.println("Stator Current Limit for "+name+" seems a little high");
            System.out.println("Make sure to lower it after you're done testing whatever it is you're testing!");
        }



        /** Now start the process of applying the configs */



        /** Wait up to 0.25 seconds for an error code after each config attempt. */
        super.getConfigurator().DefaultTimeoutSeconds = 0.25;

        while(true) {
            /** Attempt to apply configs */
            StatusCode configStatus = super.getConfigurator().apply(config);

            /** Read back the configs that are actually in use */
            TalonFXConfiguration configsInUse = new TalonFXConfiguration();
            super.getConfigurator().refresh(configsInUse);

            /** Verify all configs match their expected values.
             *  Note that we have to compare the configs using
             *  their String representation, because CTRE
             *  did not override the .equals() method for
             *  the TalonFXConfiguration type.
             */
            if (configStatus.isOK() && config.toString().equals(configsInUse.toString())) {
                System.out.println("Successfully configured "+name);
                return;
            }

            /** Log the failure if it occurred */
            String errorMessage = "Error encounted while configuring "+name;
            String errorName = "Error name: " + configStatus.getName();
            String errorDescription = "Error description: " + configStatus.getDescription();
            System.out.println(errorMessage+"\n"+errorName+"\n"+errorDescription+"\n"+"Retrying config...");
        }
    }
}
