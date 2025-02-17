
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.RobotConstants;

public class FlipperSubsystem extends SubsystemBase {
  private final Solenoid gripper = new Solenoid(PneumaticsModuleType.REVPH, 6);
  private final Solenoid flipper = new Solenoid(PneumaticsModuleType.REVPH, 9);
  private final DigitalInput coralDetector = new DigitalInput(0);

  public boolean hasCoral = false;
  private boolean isGripped = false;
  private boolean isInScoringPosition = false;

  /** Creates a new FlipperSubsystem. */
  public FlipperSubsystem() {
  }

  public void grip() {
    gripper.setPulseDuration(1.0);
    hasCoral = coralDetector.get();
    gripper.set(hasCoral);
    isGripped = gripper.get();
  }

  public boolean updateHasCoral() {
    hasCoral = coralDetector.get();
    return hasCoral;
  }

  public void letGo() {
    gripper.setPulseDuration(1.0);
    gripper.set(false);
    isGripped = false;
  }

  public void extend() {
    flipper.setPulseDuration(1.0);
    flipper.set(true);
    isInScoringPosition = true;
  }

  public void retract() {
    flipper.setPulseDuration(1.0);
    flipper.set(false);
    isInScoringPosition = false;
  }

  public void score() {
    try {
      Thread.sleep(RobotConstants.kScoreTimeoutMilliseconds);
    } catch (InterruptedException e) {
      Thread.currentThread().interrupt();
    }
    hasCoral = false;
  }

  public boolean getHasCoral() {
    return hasCoral;
  }

  @Override
  public void periodic() {
    // Scan for coral using DigitalInput.
    // If coral found, grip it after 200ms.
    if (gripper.get() == false) {
      if (coralDetector.get() == true) {
        try {
          Thread.sleep(RobotConstants.kGripperDelayMilliseconds);
        } catch (InterruptedException e) {
          Thread.currentThread().interrupt();
        }
        gripper.set(true);
      }
    }

    SmartDashboard.putBoolean("Has Coral?", hasCoral);
    SmartDashboard.putBoolean("Is Gripped?", isGripped);
    SmartDashboard.putBoolean("In Scoring Position?", isInScoringPosition);
  }

  @Override
  public void simulationPeriodic() {
    // Scan for coral using DigitalInput.
    // If coral found, grip it after 200ms.
    if (gripper.get() == false) {
      if (coralDetector.get() == true) {
        try {
          Thread.sleep(RobotConstants.kGripperDelayMilliseconds);
        } catch (InterruptedException e) {
          Thread.currentThread().interrupt();
        }
        gripper.set(true);
      }
    }

    SmartDashboard.putBoolean("Has Coral?", hasCoral);
    SmartDashboard.putBoolean("Is Gripped?", isGripped);
    SmartDashboard.putBoolean("In Scoring Position?", isInScoringPosition);
  }
}
