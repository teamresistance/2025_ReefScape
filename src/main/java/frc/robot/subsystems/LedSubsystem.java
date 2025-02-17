package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.HardwareConstants;
import frc.robot.Constants.HardwareConstants.LedMode;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class LedSubsystem extends SubsystemBase {
    AddressableLED ledstrip = new AddressableLED(HardwareConstants.kLed_portNumber);
    int length = RobotConstants.kLedLength;
    int animationFrame = 0;
    int animationDelay = RobotConstants.kLedAnimationDelayMilliseconds;
    int delayTracker = 0;

    public LedMode mode = HardwareConstants.LedMode.kOFF;

    public LedSubsystem() {
        ledstrip.setLength(length);
        ledstrip.start();
    }

    public void setLEDColor(int[] color) {
        AddressableLEDBuffer buffer = new AddressableLEDBuffer(length);
        for (var i = 0; i < buffer.getLength(); i++) {
            buffer.setRGB(i, color[0], color[1], color[2]);
        }
        ledstrip.setData(buffer);
    }

    public void strobeBetween(int frame, int[]... colors) {
        int cycleLength = length / (colors.length - 1); // Avoid out-of-bounds
        AddressableLEDBuffer buffer = new AddressableLEDBuffer(length);

        for (int i = 0; i < length; i++) {
            int shiftedIndex = (i + frame) % length; // Shift the gradient by 'frame' Leds

            int colorsBetween = shiftedIndex / cycleLength;
            int nextColor = Math.min(colorsBetween + 1, colors.length - 1); // Prevent out-of-bounds

            double ratio = (shiftedIndex % cycleLength) / (double) cycleLength; // Use double for smooth interpolation

            int red = (int) (colors[colorsBetween][0] * (1 - ratio) + colors[nextColor][0] * ratio);
            int grn = (int) (colors[colorsBetween][1] * (1 - ratio) + colors[nextColor][1] * ratio);
            int blu = (int) (colors[colorsBetween][2] * (1 - ratio) + colors[nextColor][2] * ratio);

            buffer.setRGB(i, red, grn, blu);
        }
    }

    public void turnOff() {
        AddressableLEDBuffer buffer = new AddressableLEDBuffer(length);
        for (var i = 0; i < buffer.getLength(); i++) {
            buffer.setRGB(i, 0, 0, 0);
        }
        ledstrip.setData(buffer);
    }

    public void setMode(LedMode changeTo) {
        mode = changeTo;
    }

    @Override
    public void periodic() {
        delayTracker++;

        if (delayTracker * 20 >= animationDelay) {
            animationFrame = (animationFrame + 1) % length;
            delayTracker = 0;
        }

        switch (mode) {
            case kSOLID:
                setLEDColor(RobotConstants.kLedSolidColor);
                break;
            case kSTROBE:
                strobeBetween(animationFrame,
                        RobotConstants.kLedStrobeColor1,
                        RobotConstants.kLedStrobeColor2,
                        RobotConstants.kLedStrobeColor3);
                break;
            case kOFF:
                turnOff();
                break;
        }

    }

    @Override
    public void simulationPeriodic() {

    }
}
