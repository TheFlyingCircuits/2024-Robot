package frc.robot.commands.leds;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.leds.LEDs;
import frc.robot.subsystems.shooter.Shooter;

public class ShooterChargeUp extends Command {
    AddressableLED leds;
    AddressableLEDBuffer buffer;
    Shooter shooter;
    double desiredMaxLeftFlywheelSpeed;

    public ShooterChargeUp(LEDs led, Shooter shooter, double desiredMaxLeftFlywheelSpeed) {
        this.leds = led.getLEDs();
        this.buffer = led.getBuffer();
        this.shooter = shooter;
        this.desiredMaxLeftFlywheelSpeed = desiredMaxLeftFlywheelSpeed;
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
        double leftFlywheelMetersPerSecond = shooter.getLeftFlywheelsMetersPerSecond();
        leftFlywheelMetersPerSecond = MathUtil.clamp(leftFlywheelMetersPerSecond, 0., 27.);
        int expectedColor = (int)(510 * Math.sin(((2*Math.PI)/(desiredMaxLeftFlywheelSpeed*4)) * leftFlywheelMetersPerSecond));
        for(int i = 0; i < buffer.getLength(); i++) {
            if(expectedColor <= 255 && expectedColor >= 0) {
                buffer.setRGB(i, 255, expectedColor, 0);
            } else if(expectedColor <= 510 && expectedColor >= 0) {
                buffer.setRGB(i, 510 - expectedColor, 255, 0);
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
