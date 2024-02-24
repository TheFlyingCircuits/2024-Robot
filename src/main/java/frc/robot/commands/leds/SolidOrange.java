package frc.robot.commands.leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.leds.LEDs;

public class SolidOrange extends Command {

    AddressableLED leds;
    AddressableLEDBuffer buffer;

    public SolidOrange(LEDs led) {
        this.leds = led.getLEDs();
        this.buffer = led.getBuffer();
    }

    @Override
    public void initialize() {
        for(int i = 0; i < buffer.getLength(); i++) {
            buffer.setRGB(i, 255, 135, 0);
        }
        leds.setData(buffer);
    }

    @Override
    public void execute() {}

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
