package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.LEDConstants;

public class LEDs {

    private AddressableLED leds;
    private AddressableLEDBuffer buffer;

    public LEDs() {

        

        leds = new AddressableLED(LEDConstants.ledPWMPort);
        buffer = new AddressableLEDBuffer(LEDConstants.ledLength);

        leds.setLength(buffer.getLength());
        
        leds.setData(buffer);
        leds.start();
    };

    public void setLEDColor(LEDConstants.LEDColor color) {
        int r = 0;
        int g = 0;
        int b = 0;
        switch(color) {
            case RED:
                r=255;
                break;
            case GREEN:
                g=255;
                break;
            case ORANGE:
                r=255;
                g=140;
                break;
        }
        for(int i = 0; i < buffer.getLength(); i++){
            buffer.setRGB(i, r,g,b);
        }

        leds.setData(buffer);
    }

    public AddressableLED getLEDs() {
        return leds;
    }

    public AddressableLEDBuffer getBuffer() {
        return buffer;
    }

    public void chasePattern() {
        double metersPerSecond = -1.0;
        double length = 0.1;
        double radius = length/2.0;
        double meters = (metersPerSecond * Timer.getFPGATimestamp());
        double upperBound = (meters + radius) % 1.0;

        // Java % doesn't restrict between 0 & 1,
        // it restricts between -1 & 1
        if (upperBound < 0) {
            upperBound += 1;
        }
        double lowerBound = (meters - radius) % 1.0;
        if (lowerBound < 0) {
            lowerBound += 1;
        }
        for (int i = 0; i < buffer.getLength(); i += 1) {
            double position = (i + 1.0) / buffer.getLength();

            boolean setMe = (upperBound >= lowerBound) && (position >= lowerBound) && (position <= upperBound);

            // wrap around the end of the strip to the begining of the strip
            setMe |= ((upperBound < lowerBound) && ( (position >= lowerBound) || (position <= upperBound) ));

            if (setMe) {
                buffer.setHSV(i, 4, 255, 255);
            }
            else {
                buffer.setHSV(i, 0, 0, 0);
            }
        }
        leds.setData(buffer);
    }
}
