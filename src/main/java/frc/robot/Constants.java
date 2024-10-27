/*
 * MIT License
 *
 * Copyright (c) 2025 Team 86
 *
 * https://github.com/teamresistance
 *
 * More details provided in license files
 */

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  // Modes
  public static final Mode currentMode = Mode.REAL;
  // Deadband
  public static final double joystick_headband = 0.15;
  // Robot dimensions and speed
  public static final double MAX_LINEAR_SPEED = Units.feetToMeters(11.339);
  public static final double TRACK_WIDTH_X =
      Units.inchesToMeters(19.57); // X is perpendicular to the front of the robot
  public static final double TRACK_WIDTH_Y =
      Units.inchesToMeters(24.5); // Y is parallel to the front of the robot
  public static final double DRIVE_GEAR_RATIO = 60.0 / 9.0;
  public static final double TURN_GEAR_RATIO = 396.0 / 35.0; // (24.0 / 8) * (72.0 / 14);
  // Motor and encoder constants
  public static final int DRIVE_SPARK_MAX_FL = 24;
  public static final int TURN_SPARK_MAX_FL = 25;
  public static final int CANCODER_FL = 33;
  public static final double ABSOLUTE_ENCODER_OFFSET_FL = 0.668;
  public static final int DRIVE_SPARK_MAX_FR = 26;
  public static final int TURN_SPARK_MAX_FR = 27;
  public static final int CANCODER_FR = 30;
  public static final double ABSOLUTE_ENCODER_OFFSET_FR = 0.780 - 0.5;
  public static final int DRIVE_SPARK_MAX_BL = 23;
  public static final int TURN_SPARK_MAX_BL = 22;
  public static final int CANCODER_BL = 32;
  public static final double ABSOLUTE_ENCODER_OFFSET_BL = 0.054;
  public static final int DRIVE_SPARK_MAX_BR = 21;
  public static final int TURN_SPARK_MAX_BR = 20;
  public static final int CANCODER_BR = 31;
  public static final double ABSOLUTE_ENCODER_OFFSET_BR = 0.646 + 0.5;
  // Gyro
  public static final int PIGEON2_CAN_ID = 26;
  // Odometry
  public static final double ODOMETRY_FREQUENCY = 250.0;
  public static final double WHEEL_RADIUS = Units.inchesToMeters(2.05) * 1 / 1.1132075472;
  public static final boolean FIELD_MIRROR = false;

  // Tuning
  public static boolean tuningMode = false;
  public static double drivekP = 0.001;
  public static double drivekD = 0.0;
  public static double ffkS = 0.03;
  public static double ffkV = 0.2;
  public static double turnkP = 3.5;
  public static double turnkD = 0.0;

  public enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static class AutoConstants {
    public static double kPThetaController = 5.0;
    public static double kPXController = 1.0;
    public static double kPYController = 1.0;
  }
}
