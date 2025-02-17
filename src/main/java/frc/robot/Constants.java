// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//Hardware imports
import edu.wpi.first.wpilibj.PneumaticsModuleType;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static enum JoystickType {
      k3Joysticks,
      k2JoysticksAndReefSelector,
      kNeo,
      kKybd1
    }

    public static final int klvl2Button_3Joysticks_ID = 3;
    public static final int klvl3Button_3Joysticks_ID = 4;
    public static final int klvl4Button_3Joysticks_ID = 6;
    public static final int kSelectBranchAndAddButton_3Joysticks_ID = 1;
    public static final int kclimbButton_3Joysticks_ID = 2;

    public static final int kclimbButton_2JoysticksAndReefSelector_ID = 2;

    public static final int klvl2Button_Neo_ID = 1;
    public static final int klvl3Button_Neo_ID = 2;
    public static final int klvl4Button_Neo_ID = 3;
    public static final int kSelectBranchAndAddButton_Neo_ID = 4;
    public static final int kclimbButton_Neo_ID = 5;

    public static final int klvl2Button_Kybd1_ID = 0;
    public static final int klvl3Button_Kybd1_ID = 0;
    public static final int klvl4Button_Kybd1_ID = 0;
    public static final int kSelectBranchAndAddButton_Kybd1_ID = 0;

  }

  public static class RobotConstants {
    public static final int kScoreTimeoutMilliseconds = 3000; // Milliseconds
    public static final int kGripperDelayMilliseconds = 200; // Milliseconds

    public static final String limelightName = "limelight";
    public static final double kLimelightWindowResolutionWidthPixels = 960;
    public static final double kLimelightHorizontalFOVdegrees = 62.5;

    public static final double[] kCameraToCenterOffsetInches = { 0, 0 }; // {x, y}
    public static final double[] kFlipperToCenterOffsetInches = { 0, 0 }; // {x, y}

    // These are the error thresholds for the robot to be considered aligned with
    // the reef branch.
    // Treat them as if they have a +/- tolerance.
    public static final double kXdirectionErrorThresholdInches = 1.000;
    public static final double kYdirectionErrorThresholdInches = 1.000;

    public static final int kLedLength = 60;
    public static final int kLedAnimationDelayMilliseconds = 100; // Can only be multiple of 20ms.
    public static final int[] kLedStrobeColor1 = { 255, 255, 0 };
    public static final int[] kLedStrobeColor2 = { 0, 255, 0 };
    public static final int[] kLedStrobeColor3 = { 0, 255, 255 };
    public static final int[] kLedSolidColor = { 132, 76, 130 };
  }

  public static class HardwareConstants {
    public static final PneumaticsModuleType pneumaticsModuleType = PneumaticsModuleType.CTREPCM;

    // Arm
    public static final int kSolenoid_wristRotator_portNumber = 2;
    public static final int kWristRotatorPulseDurationSeconds = 1;
    public static final int kSolenoid_armLifter_portNumber = 0;
    public static final int kArmLifterPulseDurationSeconds = 1;

    // Elevator
    public static final int kSolenoid_firstStage_portNumber = 1;
    public static final int kFirstStagePulseDurationSeconds = 1;
    public static final int kSolenoid_secondStage_portNumber = 5;
    public static final int kSecondStagePulseDurationSeconds = 1;

    // Climber
    public static final int kClimberSolenoid_portNumber = 4;

    // Led
    public static final int kLed_portNumber = 0;

    public enum LedMode {
      kSOLID,
      kSTROBE,
      kOFF
    }

    // Others
    public static final int pwmTalonFX_channel = 0;
  }

  public static class FieldConstants {
    public static final double kReefBranchWidthInches = 1.660;
    public static final double kReefBranchInsetInches = 1.125;

    public static final double[] branchAposition = { 10.50, 13.00 };
    public static final double[] branchBposition = { 10.50, 12.13 };
    public static final double[] branchCposition = { 12.80, 9.44 };
    public static final double[] branchDposition = { 13.73, 8.90 };
    public static final double[] branchEposition = { 17.02, 9.65 };
    public static final double[] branchFposition = { 17.96, 10.19 };
    public static final double[] branchGposition = { 18.96, 13.42 };
    public static final double[] branchHposition = { 18.96, 14.50 };
    public static final double[] branchIposition = { 16.66, 16.98 };
    public static final double[] branchJposition = { 15.73, 17.52 };
    public static final double[] branchKposition = { 12.43, 16.77 };
    public static final double[] branchLposition = { 11.50, 16.23 };

    public static final double[] reefABmidpoint = { 10.50, 12.46 };
    public static final double reefABangle = Math.PI;
    public static final double[] reefCDmidpoint = { 13.26, 9.17 };
    public static final double reefCDangle = 4.0 / 3.0 * Math.PI;
    public static final double[] reefEFmidpoint = { 17.49, 9.92 };
    public static final double reefEFangle = 5.0 / 3.0 * Math.PI;
    public static final double[] reefGHmidpoint = { 18.96, 13.96 };
    public static final double reefGHangle = 0.0;
    public static final double[] reefIJmidpoint = { 16.19, 17.25 };
    public static final double reefIJangle = 1.0 / 3.0 * Math.PI;
    public static final double[] reefKLmidpoint = { 11.97, 16.50 };
    public static final double reefKLangle = 2.0 / 3.0 * Math.PI;
  }

}