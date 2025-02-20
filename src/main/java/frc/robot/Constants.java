// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {

  // Solenoids
  public static final PneumaticsModuleType SolenoidModuleType = PneumaticsModuleType.CTREPCM;
  public static final int kCentererSolenoidChannel = 0;
  public static final int kElevatorSolenoid1Channel = 1;
  public static final int kElevatorSolenoid2Channel = 2;
  public static final int kGripperSolenoidChannel = 3;
  public static final int kFlipperSolenoidChannel = 4;
  public static final int kClimberSolenoidChannel = 4;

  // Drive
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : Mode.SIM;
  public static final boolean tuningMode = false;

  // LED Strip
  public static final int kLedLength = 60;
  public static final int kLedAnimationDelayMilliseconds = 100; // Can only be multiple of 20ms.
  // has coral
  public static final int[] kLedStrobeColor1 = { 0, 0, 255 };
  public static final int[] kLedStrobeColor2 = { 0, 255, 255 };
  public static final int[] kLedStrobeColor3 = { 0, 255, 255 };
  // solid
  public static final int[] kLedSolidColor = { 132, 76, 130 };
  // is climbing
  public static final int[] kLedStrobeColor4 = { 50, 100, 50 };
  public static final int[] kLedStrobeColor5 = { 100, 200, 100 };
  public static final int[] kLedStrobeColor6 = { 50, 200, 50 };
  public static final int kLed_portNumber = 0;
  public enum LedMode {
    kSOLID,
    kSTROBE,
    kOFF
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
