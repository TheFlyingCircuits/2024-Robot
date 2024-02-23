package frc.robot.commands.leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.leds.LEDs;

public class ChasePattern extends Command {

    double metersPerSecond = -1.0;
    double length = 0.1;
    double radius = length/2.0;
    double meters;
    double upperBound;
    AddressableLED leds;
    AddressableLEDBuffer buffer;

    public ChasePattern(LEDs led) {
        this.leds = led.getLEDs();
        this.buffer = led.getBuffer();
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        meters = (metersPerSecond * Timer.getFPGATimestamp());
        upperBound = (meters + radius) % 1.0;
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
                buffer.setRGB(i, 255, 0, 0);
            }
            else {
                buffer.setRGB(i, 0, 0, 0);
            }
        }
        leds.setData(buffer);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        for(int i = 0; i < buffer.getLength(); i++) {
            buffer.setHSV(i, 0, 0, 0);
        }
        leds.setData(buffer);
    }

}
