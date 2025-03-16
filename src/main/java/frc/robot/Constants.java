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

  // Elevator / Flipper / Gripper
  public static final double SECONDS_TO_RAISE_ELEVATOR = 0;
  public static final double SECONDS_TO_SCORE = 0;
  public static final double SECONDS_GRIPPER_DELAY = 0;
  public static final int CORAL_SENSOR_CHANNEL_1 = 0;
  public static final int CORAL_SENSOR_CHANNEL_2 = 1;
  public static final double SECONDS_PER_CENTERING_ATTEMPT = 0.7;
  public static final double SECONDS_OF_CENTERING_PULSE = 0.5;

  // Interface
  public enum InterfaceExecuteMode {
    REEF,
    CORAL,
    CLIMBER,
    EXECUTE
  }

  // Solenoids / Pressure
  public static final PneumaticsModuleType SOLENOID_MODULE_TYPE = PneumaticsModuleType.CTREPCM;
  public static final int PRESSURE_HUB_ID = 2;
  public static final int PRESSURE_SENSOR_ID = 0;
  public static final int COMPRESSOR_RELAY_ID = 0;
  public static final int CENTERER_SOLENOID_CHANNEL = 0;
  public static final int ELEVATOR_SOLENOID1_CHANNEL = 1;
  public static final int ELEVATOR_SOLENOID2_CHANNEL = 2;
  public static final int GRIPPER_SOLENOID_CHANNEL = 3;
  public static final int FLIPPER_SOLENOID_CHANNEL = 4;
  public static final int CLIMBER_SOLENOID_CHANNEL = 5;

  public enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }
}
