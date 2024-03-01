package frc.robot.commands.leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.leds.LEDs;

public class RedBlueChasePattern extends Command {
    
    int length = 4;
    int offset = 0;
    AddressableLED leds;
    AddressableLEDBuffer buffer;

    public RedBlueChasePattern(LEDs led) {
        this.leds = led.getLEDs();
        this.buffer = led.getBuffer();
    }

    @Override
    public void initialize() {
        for(int i = 0; i < buffer.getLength(); i++) {
            buffer.setRGB(i, 0, 0, 0);
        }
        leds.setData(buffer);
    }

    @Override
    public void execute() { 

        int counter = 0;
        boolean isColor = true;
        boolean color = true;

        for(int i = 0; i < buffer.getLength(); i++) { 
            if(counter >= 60 - (length*2)) {
                counter = 0;
                isColor = true;
            }else if(counter >= length) {
                counter = 0;
                isColor = false;
            }
            if(isColor) {
                if(color) {
                    buffer.setRGB(i, 0, 0, 255);
                } else {
                    buffer.setRGB(i, 255, 0, 0);
                }
            } else {
                buffer.setRGB(i, 0, 0, 0);
            }
            counter++;
        }

        offset++;

        leds.setData(buffer);
    }

    @Override
    public void end(boolean interrupted) {
        for(int i = 0; i < buffer.getLength(); i++) {
            buffer.setHSV(i, 0, 0, 0);
        }
        leds.setData(buffer);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
