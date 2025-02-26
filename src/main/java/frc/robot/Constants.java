package frc.robot;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode CURRENT_MODE = RobotBase.isReal() ? Mode.REAL : Mode.SIM;
  public static final boolean TUNING_MODE = false;

  // Solenoids
  public static final PneumaticsModuleType SOLENOID_MODULE_TYPE = PneumaticsModuleType.CTREPCM;
  public static final int CENTERER_SOLENOID_CHANNEL = 0;
  public static final int ELEVATOR_SOLENOID1_CHANNEL = 1;
  public static final int ELEVATOR_SOLENOID2_CHANNEL = 2;
  public static final int GRIPPER_SOLENOID_CHANNEL = 3;
  public static final int FLIPPER_SOLENOID_CHANNEL = 4;
  public static final int CLIMBER_SOLENOID_CHANNEL = 4;

  // Reef
  public static final int TIME_TO_RAISE_STAGE = 0;
  public static final int TIME_TO_REACH_REEF = 0;
  public static final int TIME_TO_SHIFT_LEFTRIGHT = 0;

  // LED Strip
  public static final int LED_LENGTH = 60;
  public static final int LED_ANIMATION_DELAY_MS = 100; // Can only be multiple of 20ms.
  // Coral Indicators
  public static final int[] LED_STROBE_COLOR1 = {0, 0, 255};
  public static final int[] LED_STROBE_COLOR2 = {0, 255, 255};
  public static final int[] LED_STROBE_COLOR3 = {0, 255, 255};
  // Is Climbing
  public static final int[] LED_STROBE_COLOR4 = {50, 100, 50};
  public static final int[] LED_STROBE_COLOR5 = {100, 200, 100};
  public static final int[] LED_STROBE_COLOR6 = {50, 200, 50};
  // Solid
  public static final int[] LED_COLOR_SOLID = {132, 76, 130};
  public static final int LED_PORT_NUMBER = 0;

  public enum LedMode {
    SOLID,
    STROBE,
    OFF
  }

  public enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }
}
