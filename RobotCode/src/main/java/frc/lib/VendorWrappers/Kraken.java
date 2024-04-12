package frc.lib.VendorWrappers;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Timer;
import frc.lib.MotorTempObject;
import frc.lib.subsystem.Fault;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

/**
 * A wrapper class around TalonFX that provides some convience features,
 * like auto-retyring configs if they fail.
 */
public class Kraken extends TalonFX {

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

    public List<Fault> autoDiagnoseIsAtTargetRPS(double expectedRPS, double tolerance, boolean isForward) {
        ArrayList<Fault> faults = new ArrayList<Fault>();

        String direction = isForward ? "forward" : "backward";

        if (Math.abs(this.getVelocity().getValueAsDouble()) < .1) {
            new Fault("[Auto Diagnose] "+name+" not moving " +direction, false);
        } else if (Math.abs((this.getVelocity().getValueAsDouble())) > expectedRPS+tolerance) {
            new Fault("[Auto Diagnose] "+name+" moving "+direction+" too fast", false);
        } else if (Math.abs((this.getVelocity().getValueAsDouble())) < expectedRPS-tolerance) {
            new Fault("[Auto Diagnose] "+name+" moving "+direction+" too slow", false);
        }
        return faults;
    }

    public MotorTempObject getMotorTempObject() {
        return new MotorTempObject(
            this.name,
            this.getDeviceTemp().getValueAsDouble()
        );
    }


    /** 
     * Returns the name assigned at motor initialization.
     * It is how we should refer to this motor in error messages.
     * <p>
     * This specific method is useful for the automated diagnostic stuff to run stuff in paralell 
     */
    public String getName() {
        return this.name;
    }
}
