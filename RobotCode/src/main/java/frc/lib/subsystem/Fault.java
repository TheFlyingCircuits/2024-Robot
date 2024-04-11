package frc.lib.subsystem;

import edu.wpi.first.wpilibj.Timer;

public class Fault {
    public final String message;
    public final double timestamp;
    public final boolean isJustWarning;

    public Fault(String message, boolean isJustWarning) {
        this.message = message;
        this.timestamp = Timer.getFPGATimestamp();
        this.isJustWarning = isJustWarning;
        System.out.println("fault created: " + message + " | " + isJustWarning);
    }

    @Override
    public boolean equals(Object other) {
        // return true if its the same fault object
        if (other == this) return true;

        // if statement is true if the other object is a Fault
        if(other instanceof Fault) {
            // casts the other object to a Fault object
            Fault oth = (Fault) other;

            // returns true if the message and warnings are the same
            return message.equals(oth.message) && (isJustWarning == oth.isJustWarning);
        }
        
        return false;
    }
}