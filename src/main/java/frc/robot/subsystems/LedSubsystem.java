//
// THIS WILL NOT BE USED.
//

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.LedMode;
import org.littletonrobotics.junction.Logger;

public class LedSubsystem extends SubsystemBase {

  /** An LED strip with length as specified by {@code Constants.kLedLength} */
  AddressableLED ledstrip = new AddressableLED(Constants.LED_PORT_NUMBER);

  /** The length of the LED strip */
  int length = Constants.LED_LENGTH;

  int animationFrame = 0;
  int strobeSetting = 0;

  /**
   * The amount of time to advance to the next strobe animation frame. Can only be 20ms or greater
   * multiples of 20ms. A higher value means a slower animation.
   */
  int animationDelay = Constants.LED_ANIMATION_DELAY_MS;

  int delayTracker = 0;

  /** An enum value specifying how the LED lights up. */
  public LedMode mode = Constants.LedMode.OFF;

  public LedSubsystem() {
    ledstrip.setLength(length);
    ledstrip.start();
  }

  /**
   * This method sets all LEDs to an RGB value specified by the {@code int[] color}. This array MUST
   * be of length 3.
   *
   * @param color An array starting the intended color in RGB format.
   * @return Void
   */
  public void setLEDColor(int[] color) {
    AddressableLEDBuffer buffer = new AddressableLEDBuffer(length);
    for (var i = 0; i < buffer.getLength(); i++) {
      buffer.setRGB(i, color[0], color[1], color[2]);
    }
    ledstrip.setData(buffer);
  }

  /**
   * Creates an animated gradient effect between any number of distinct colors.
   *
   * @param colors a bunch of int[]s of length 3, each a color in RGB format.
   * @return void
   */
  public void strobeBetween(int frame, int[]... colors) {
    int cycleLength = length / (colors.length - 1); // Avoid out-of-bounds
    AddressableLEDBuffer buffer = new AddressableLEDBuffer(length);

    for (int i = 0; i < length; i++) {
      int shiftedIndex = (i + frame) % length; // Shift the gradient by 'frame' Leds

      int colorsBetween = shiftedIndex / cycleLength;
      int nextColor = Math.min(colorsBetween + 1, colors.length - 1); // Prevent out-of-bounds

      double ratio =
          (shiftedIndex % cycleLength)
              / (double) cycleLength; // Use double for smooth interpolation

      int red = (int) (colors[colorsBetween][0] * (1 - ratio) + colors[nextColor][0] * ratio);
      int grn = (int) (colors[colorsBetween][1] * (1 - ratio) + colors[nextColor][1] * ratio);
      int blu = (int) (colors[colorsBetween][2] * (1 - ratio) + colors[nextColor][2] * ratio);

      buffer.setRGB(i, red, grn, blu);
    }
  }

  /** Changes what colors it's strobing between */
  public void setStrobeSetting(int setting) {
    strobeSetting = setting;
  }

  /** Turns off the led strip. */
  public void turnOff() {
    AddressableLEDBuffer buffer = new AddressableLEDBuffer(length);
    for (var i = 0; i < buffer.getLength(); i++) {
      buffer.setRGB(i, 0, 0, 0);
    }
    ledstrip.setData(buffer);
  }

  /**
   * Changes LEDSubsystem.mode.
   *
   * @param changeTo The intended mode to change to. Can only be kSOLID, kSTROBE, or kOFF.
   */
  public void setMode(LedMode changeTo) {
    mode = changeTo;
  }

  @Override
  public void periodic() {
    Logger.recordOutput("LED/Current Mode", mode);
    Logger.recordOutput("LED/Current Strobe Setting", strobeSetting);

    delayTracker++;

    if (delayTracker * 20 >= animationDelay) {
      animationFrame = (animationFrame + 1) % length;
      delayTracker = 0;
    }

    switch (mode) {
      case SOLID:
        setLEDColor(Constants.LED_COLOR_SOLID);
        break;
      case STROBE:
        switch (strobeSetting) {
          case 0:
            strobeBetween(
                animationFrame,
                Constants.LED_STROBE_COLOR1,
                Constants.LED_STROBE_COLOR2,
                Constants.LED_STROBE_COLOR3);
            break;
          case 1:
            strobeBetween(
                animationFrame,
                Constants.LED_STROBE_COLOR4,
                Constants.LED_STROBE_COLOR5,
                Constants.LED_STROBE_COLOR6);
        }
        break;
      case OFF:
        turnOff();
        break;
      default: // Do nothing
    }
  }

  @Override
  public void simulationPeriodic() {}
}
