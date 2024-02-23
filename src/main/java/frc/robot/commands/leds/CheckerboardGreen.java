package frc.robot.commands.leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.leds.LEDs;

public class CheckerboardGreen extends Command {
    
    int space = 5;
    AddressableLED leds;
    AddressableLEDBuffer buffer;
    boolean isActive = true;
    double timeToHold = 0.5;

    public CheckerboardGreen(LEDs led) {
        this.leds = led.getLEDs();
        this.buffer = led.getBuffer();
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        for(int i = 0; i < buffer.getLength(); i++) {
            if(i % space == 0){
            isActive = !isActive;
            }
            if(isActive){
                buffer.setRGB(i, 0, 255, 0);
            } else {
                buffer.setRGB(i, 0, 0, 0);
            }
        }
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
