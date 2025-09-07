package frc.robot.subsystems;

import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.SingleFadeAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDMode;

public class LEDSubsystem extends SubsystemBase {

  public LEDSubsystem() {}

  private final int LED_START_INDEX = 8;
  private final int LED_END_INDEX = 67;
  private final CANdle candle = new CANdle(60); // TODO: correct port
  private int psi = 120;
  private LEDMode mode = LEDMode.WHITE;
  private boolean isLocked = false;

  public void setMode(LEDMode newMode, boolean lock) {
    if (!isLocked) mode = newMode;
    if (lock) isLocked = true;
  }

  public void setMode(LEDMode newMode, boolean lock, int psi) {
    if (!isLocked) mode = newMode;
    if (lock) isLocked = true;
    this.psi = psi;
  }

  public void unlock() {
    isLocked = false;
  }

  @Override
  public void periodic() {

    // auto set led
    switch (mode) {
      case WHITE:
        candle.setControl(
            new SolidColor(LED_START_INDEX, LED_END_INDEX).withColor(new RGBWColor(255, 255, 255)));
        break;
      case CORAL_OUT:
        candle.setControl(
            new StrobeAnimation(LED_START_INDEX, LED_END_INDEX)
                .withFrameRate(4)
                .withColor(new RGBWColor(0, 255, 0)));
        break;
      case ALGAE_OUT:
        candle.setControl(
            new StrobeAnimation(LED_START_INDEX, LED_END_INDEX)
                .withFrameRate(4)
                .withColor(new RGBWColor(0, 0, 255)));
        break;
      case CORAL_IN:
        candle.setControl(
            new StrobeAnimation(LED_START_INDEX, LED_END_INDEX)
                .withFrameRate(8)
                .withColor(new RGBWColor(255, 255, 255)));
        break;
      case CLIMBING:
        candle.setControl(new RainbowAnimation(LED_START_INDEX, LED_END_INDEX).withFrameRate(60));
      case AIR_GOOD: // >70
        candle.setControl(
            new SingleFadeAnimation(LED_START_INDEX, LED_END_INDEX)
                .withColor(new RGBWColor(186, 85, 211))
                .withFrameRate(2 * psi));
      case AIR_LOW: // 40-70
        candle.setControl(
            new SingleFadeAnimation(LED_START_INDEX, LED_END_INDEX)
                .withColor(new RGBWColor(255, 255, 0))
                .withFrameRate(4 * psi));
      case AIR_BAD: // <40
        candle.setControl(
            new SingleFadeAnimation(LED_START_INDEX, LED_END_INDEX)
                .withColor(new RGBWColor(200, 0, 0))
                .withFrameRate(8 * psi));
    }
  }
}
