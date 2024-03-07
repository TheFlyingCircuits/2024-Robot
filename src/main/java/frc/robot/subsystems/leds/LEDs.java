package frc.robot.subsystems.leds;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

public class LEDs extends SubsystemBase {

    private AddressableLED leds;
    private AddressableLEDBuffer buffer;

    /**
     * TODO: document whole class
     */
    public LEDs() {
        leds = new AddressableLED(LEDConstants.ledPWMPort);
        buffer = new AddressableLEDBuffer(LEDConstants.ledsPerStrip);

        leds.setLength(buffer.getLength());
        
        leds.setData(buffer);
        leds.start();
    }

    public void solidColorHSV(int h, int s, int v) {
        for (int i = 0; i < buffer.getLength(); i += 1) {
            buffer.setHSV(i, h, s, v);
        }
        leds.setData(buffer);
    }

    public void turnOff() {
        this.solidColorHSV(0, 0, 0);
    }

    public void slingshot(double progress) {
        double maxOrange = LEDConstants.stripLengthMeters + (LEDConstants.metersPerLed/2) - (LEDConstants.stripLengthMeters/2)*progress*2;
        double minOrange = 0 - (LEDConstants.metersPerLed/2) + (LEDConstants.stripLengthMeters/2)*progress*2;

        double maxOff = maxOrange + LEDConstants.stripLengthMeters/2;
        double minOff = minOrange - LEDConstants.stripLengthMeters/2;
        int allianceHue = this.getAllianceHue();
        for (int i = 0; i < buffer.getLength(); i += 1) {
            double position = (i + 1) * LEDConstants.metersPerLed;

            if (position <= maxOrange && position >= minOrange) {
                buffer.setHSV(i, LEDConstants.Hues.orangeSignalLight, 255, 255);
            }
            else if (position <= maxOff && position >= minOff) {
                buffer.setRGB(i, 0, 0, 0);
            }
            else {
                buffer.setHSV(i, allianceHue, 255, 255);
            }
        }
        leds.setData(buffer);
    }

    public void showArmProgress(double progress) {
        // Show arm progress on the top 3rd of the LED strip
        double progressBarStartLocation = LEDConstants.stripLengthMeters + (LEDConstants.metersPerLed/2);
        double progressBarEndLocation = LEDConstants.topThirdBreakpoint;
        this.showProgressAsTrafficLight(progressBarStartLocation, progressBarEndLocation, progress);
    }

    public void showFlywheelProgress(double progress) {
        // Show flywheel progress on the middle 3rd of the LED strip (building progress towards the middle)
        double topHalfStartLocation = LEDConstants.topThirdBreakpoint;
        double bottomHalfStartLocation = LEDConstants.bottomThirdBreakpoint;
        double endLocation = LEDConstants.stripLengthMeters/2.0; // end in middle of strip.
        this.showProgressAsTrafficLight(topHalfStartLocation, endLocation, progress);
        this.showProgressAsTrafficLight(bottomHalfStartLocation, endLocation, progress);
    }

    public void showDrivetrainProgress(double progress) {
        // Show drivetrain progress on the bottom 3rd of the led strips
        double progressBarStartLocation = 0 - (LEDConstants.metersPerLed/2);
        double progressBarEndLocation = LEDConstants.bottomThirdBreakpoint;
        this.showProgressAsTrafficLight(progressBarStartLocation, progressBarEndLocation, progress);
    }

    /**
     * Draws a color gradient (red -> yellow -> green) on a segment of the LED strip.
     * Intended to be used as a progress-bar-esque indicator for things like
     * flywheel spinup and pivot/drivetrain alignment to a goal.
     * 
     * @param startPositionMeters The position on the LED strip (measured in meters) where the pattern will start (this will be red)
     *                            A position of 0 is the part of the strip that's closest to the strip's power cable.
     *                            A position of stripLengthMeters will be the other end of the strip.
     * 
     * @param endPositionMeters The posiion on the LED strip (measured in meters) where the pattern will end (this will be green)
     *                          A position of 0 is the part of the strip that's closest to the strip's power cable.
     *                          A position of stripLengthMeters will be the other end of the strip.
     * 
     * @param progress A number in the range [0, 1] that indicates how much of the pattern should be drawn.
     *                 0 will have no lights on, 0.5 will have some red and yellow, 1.0 will have the full red -> yellow -> green pattern
     *                 (it will look like a traffic light with all of the lights on.)
     */
    public void showProgressAsTrafficLight(double startPositionMeters, double endPositionMeters, double progress) {
        double startHue = LEDConstants.Hues.redTrafficLight;
        double endHue = LEDConstants.Hues.greenTrafficLight;

        // iterate through all LEDs on the strip
        for (int i = 0; i < buffer.getLength(); i += 1) {

            // Find how far we are along the strip
            double positionAlongStrip = (i + 1) * LEDConstants.metersPerLed;

            // See where the current LED lies within the progress bar segment.
            // 0 = beginning of segment, 1 = end of segment
            double positionWithinSegment = (positionAlongStrip - startPositionMeters) / (endPositionMeters - startPositionMeters);

            // leave the current LED alone if it's not in the segment.
            if (positionWithinSegment < 0 || positionWithinSegment > 1) {
                continue;
            }

            // turn off the LED if it's in the segment,
            // but the progress bar hasn't reached it yet.
            if (positionWithinSegment > progress) {
                buffer.setRGB(i, 0, 0, 0);
                continue;
            }

            // Color the LED based on it's position within the segment, as well as the given progress.
            // Squaring positionWithinSegent was experimentally found to give more aesthetically
            // pleasing results than simply lerping from startHue to endHue.
            double hue = (endHue - startHue) * Math.pow(positionWithinSegment, 2) + startHue;
            buffer.setHSV(i, (int)hue, 255, 255);
        }
        leds.setData(buffer);
    }

    public void loadingPattern() {
        // use negative speed to move from the top of the strip to the bottom of the strip.
        double patternSpeedMetersPerSecond = -1.0;
        int numBlobs = 3;
        double blobLength = (LEDConstants.stripLengthMeters/4) / numBlobs; // a quarter of the strip will be lit up at any given moment.
        double blobRadius = blobLength / 2;

        // Start with a blank strip
        for (int i = 0; i < buffer.getLength(); i += 1) {
            buffer.setRGB(i, 0, 0, 0);
        }

        // draw each blob
        double blobCenter = this.wrapPosition(patternSpeedMetersPerSecond * Timer.getFPGATimestamp());
        for (int blobIndex = 0; blobIndex < numBlobs; blobIndex += 1) {
            double upperBound = this.wrapPosition(blobCenter + blobRadius);
            double lowerBound = this.wrapPosition(blobCenter - blobRadius);

            // find which LEDs are within the current blob
            for (int ledIndex = 0; ledIndex < buffer.getLength(); ledIndex += 1) {
                double position = (ledIndex + 1) * LEDConstants.metersPerLed;

                boolean partOfBlob = (lowerBound <= upperBound) && (position >= lowerBound) && (position <= upperBound);
                
                // wrap around from the end of the strip to the beginning of the strip
                partOfBlob |= (upperBound < lowerBound) && ( (position >= lowerBound) || (position <= upperBound) );

                if (partOfBlob) {
                    buffer.setHSV(ledIndex, LEDConstants.Hues.orangeSignalLight, 255, 255);
                }
            }

            // Move to the next blob
            blobCenter = this.wrapPosition(blobCenter + (LEDConstants.stripLengthMeters / numBlobs));
        }

        leds.setData(buffer);
    }

    private double wrapPosition(double positionToWrap) {
        // https://stackoverflow.com/questions/5385024/mod-in-java-produces-negative-numbers
        double wrappedPosition = positionToWrap % LEDConstants.stripLengthMeters;
        if (wrappedPosition < 0) {
            wrappedPosition += LEDConstants.stripLengthMeters;
        }
        return wrappedPosition;
    }

    // public Command showAimProgressCommand(DoubleSupplier flywheelProgress, DoubleSupplier armProgress, DoubleSupplier drivetrainProgress) {
    //     return this.run(() -> {
    //         this.showFlywheelProgress(flywheelProgress.getAsDouble());
    //         this.showArmProgress(armProgress.getAsDouble());
    //         this.showDrivetrainProgress(drivetrainProgress.getAsDouble());
    //     });
    // }

    public int getAllianceHue() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (!alliance.isPresent() || !DriverStation.isDSAttached()) {
            return LEDConstants.Hues.betweenBlueAndRed;
        }
        else if (alliance.get() == Alliance.Blue) {
            return LEDConstants.Hues.blueBumpers;
        }
        else if (alliance.get() == Alliance.Red) {
            return LEDConstants.Hues.redBumpers;
        }
        return LEDConstants.Hues.betweenBlueAndRed;
    }

    public Command allianceColorHeartbeat() {
        return super.run(() -> {
            double period = 1.5;
            double timeBetweenBeats = 0.25;
            double progress = Timer.getFPGATimestamp() % period;
            double k1 = 8;
            double k2 = 2;

            double normalizedBrightness = 0;
            if (progress < timeBetweenBeats) {
                normalizedBrightness = Math.exp(-k1*progress);
            }
            else {
                normalizedBrightness = Math.exp(-k2*(progress - timeBetweenBeats));
            }

            int value = (int)(255 * normalizedBrightness);
            this.solidColorHSV(this.getAllianceHue(), 255, value);
        });
    }

    public Command playIntakeAnimationCommand() {
        return super.run(this::loadingPattern);
    }

    public Command solidColorCommand(int h, int s, int v) {
        return super.run(() -> {this.solidColorHSV(h, s, v);});
    }

    public Command solidOrangeCommand() {
        return this.solidColorCommand(LEDConstants.Hues.orangeSignalLight, 255, 255);
    }

    public Command turnOffCommand() {
        return super.run(this::turnOff);
    }

    public Command flashWhiteCommand(int numFlashes, double totalRuntimeSeconds) {
        double onTime = (totalRuntimeSeconds / numFlashes) / 2;
        double offTime = onTime;

        Command output = new InstantCommand();
        for (int i = 0; i < numFlashes; i += 1) {
            output = output.andThen(this.solidColorCommand(0, 0, 255).withTimeout(onTime))
                           .andThen(this.turnOffCommand().withTimeout(offTime));
        }

        return output;
    }

    public AddressableLEDBuffer getBuffer() {
        return buffer;
    }

    public AddressableLED getLEDs() {
        return leds;
    }
}
