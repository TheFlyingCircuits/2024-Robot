package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import frc.robot.Constants.LEDConstants;

public class LEDs {

    private AddressableLED leds;
    private AddressableLEDBuffer buffer;

    public LEDs() {

        

        leds = new AddressableLED(LEDConstants.ledID);
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
    }
}
